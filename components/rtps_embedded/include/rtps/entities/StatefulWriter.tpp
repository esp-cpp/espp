/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/

#include "rtps/entities/StatefulWriter.h"
#include "rtps/messages/MessageFactory.h"
#include "rtps/messages/MessageTypes.h"
#include "rtps/storages/PayloadBuffer.h"
#include "rtps/utils/Log.h"
#include <chrono>
#include <cstring>
#include <stdio.h>
#include <thread>

using rtps::StatefulWriterT;

#if SFW_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.h"
#define SFW_LOG(...)                                                           \
  if (true) {                                                                  \
    printf("[Stateful Writer %s] ", this->m_attributes.topicName);             \
    printf(__VA_ARGS__);                                                       \
    printf("\r\n");                                                            \
  }
#else
#define SFW_LOG(...) //
#endif

template <class NetworkDriver>
StatefulWriterT<NetworkDriver>::~StatefulWriterT() {
  m_running = false;
  if (m_heartbeatTask) {
    m_heartbeatTask->stop();
  }
  while (m_thread_running) {
    // Wait for the heartbeat loop to observe m_running and exit.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

template <class NetworkDriver>
bool StatefulWriterT<NetworkDriver>::init(TopicData attributes,
                                          TopicKind_t topicKind,
                                          ThreadPool *threadPool,
                                          NetworkDriver &driver,
                                          bool enfUnicast) {

  if (m_mutex == nullptr) {
    if (!createMutex(&m_mutex)) {

      SFW_LOG("Failed to create mutex.\n");

      return false;
    }
  }

  m_attributes = attributes;

  mp_threadPool = threadPool;
  m_srcPort = attributes.unicastLocator.port;
  m_enforceUnicast = enfUnicast;
  m_topicKind = topicKind;

  m_nextSequenceNumberToSend = {0, 1};
  m_proxies.clear();

  m_transport = &driver;
  m_history.clear();
  m_hbCount = {1};

  if (!m_disposeWithDelay.init()) {
    SFW_LOG("Failed to initialize delayed dispose buffer mutex.\n");
    return false;
  }

  // Thread already exists, do not create new one (reusing slot case)
  m_is_initialized_ = true;

  if (!m_thread_running) {

    m_running = true;
    m_thread_running = false;

    const char *task_name = "HBThread";
    if (m_attributes.endpointGuid.entityId ==
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER) {
      task_name = "HBThreadPub";
    } else if (m_attributes.endpointGuid.entityId ==
               ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER) {
      task_name = "HBThreadSub";
    }

    if (!m_heartbeatTask) {
      espp::Task::Config config;
      config.callback = [this]() {
        sendHeartBeatLoop();
        return true;
      };
      config.task_config.name = task_name;
      config.task_config.stack_size_bytes = Config::HEARTBEAT_STACKSIZE;
      config.task_config.priority = Config::THREAD_POOL_WRITER_PRIO;
      config.log_level = espp::Logger::Verbosity::WARN;
      m_heartbeatTask = espp::Task::make_unique(config);
    }

    (void)m_heartbeatTask->start();
  }

  return true;
}

template <class NetworkDriver> void StatefulWriterT<NetworkDriver>::reset() {
  m_is_initialized_ = false;
  // TODO
}

template <class NetworkDriver>
const rtps::CacheChange *StatefulWriterT<NetworkDriver>::newChange(
    ChangeKind_t kind, const uint8_t *data, DataSize_t size, bool inLineQoS,
    bool markDisposedAfterWrite) {
  INIT_GUARD()
  if (isIrrelevant(kind)) {
    return nullptr;
  }

  Lock lock{m_mutex};
  if (!m_is_initialized_) {
    return nullptr;
  }

  if (m_history.isFull()) {
    // Right now we drop elements anyway because we cannot detect non-responding
    // readers yet. return nullptr;
    SequenceNumber_t newMin =
        ++SequenceNumber_t(m_history.getCurrentSeqNumMin());
    if (m_nextSequenceNumberToSend < newMin) {
      m_nextSequenceNumberToSend =
          newMin; // Make sure we have the correct sn to send
    }
    SFW_LOG("History full! Dropping changes %s.\r\n", this->m_attributes.topicName);
  }

  auto *result =
      m_history.addChange(data, size, inLineQoS, markDisposedAfterWrite);
  if (mp_threadPool != nullptr) {
    mp_threadPool->addWorkload(this);
  }

  SFW_LOG("Adding new data.\n");

  return result;
}

template <class NetworkDriver> void StatefulWriterT<NetworkDriver>::progress() {
  INIT_GUARD()
  Lock lock{m_mutex};
  CacheChange *next = m_history.getChangeBySN(m_nextSequenceNumberToSend);
  if (next != nullptr) {
    uint32_t i = 0;
    for (const auto &proxy : m_proxies) {
      if (!m_enforceUnicast) {
        sendDataWRMulticast(proxy, next);
      } else {
        i++;
        sendData(proxy, next);
      }
    }

    SFW_LOG("Sending data with SN %u.%u", (int)m_nextSequenceNumberToSend.low,
            (int)m_nextSequenceNumberToSend.high);

    if (next->disposeAfterWrite) {
      SFW_LOG("Dispose after write msg sent to %u proxies\r\n", (int)i);
    }

    /*
     * Use case: deletion of local endpoints
     * -> send Data Message with Disposed Flag set
     * -> Set respective SEDP CacheChange as NOT_ALIVE_DISPOSED after
     * transmission to proxies
     * -> onAckNack will send Gap Messages to skip deleted local endpoints
     * during SEDP
     */
    if (next->disposeAfterWrite) {
      next->sentTime = std::chrono::steady_clock::now();
      if (!m_disposeWithDelay.copyElementIntoBuffer(next->sequenceNumber)) {
        SFW_LOG("Failed to enqueue dispose after write!");
        m_history.dropChange(next->sequenceNumber);
      } else {
        SFW_LOG("Delayed dispose scheduled for sn %u %u\r\n",
                (int)next->sequenceNumber.high, (int)next->sequenceNumber.low);
      }
    }

    ++m_nextSequenceNumberToSend;
    sendHeartBeat();

  } else {
    SFW_LOG("Couldn't get a CacheChange with SN (%li,%lu)\n",
            m_nextSequenceNumberToSend.high, m_nextSequenceNumberToSend.low);
  }
}

template <class NetworkDriver>
void StatefulWriterT<NetworkDriver>::setAllChangesToUnsent() {
  INIT_GUARD()
  Lock lock{m_mutex};

  m_nextSequenceNumberToSend = m_history.getCurrentSeqNumMin();

  if (mp_threadPool != nullptr) {
    mp_threadPool->addWorkload(this);
  }
}

template <class NetworkDriver>
void StatefulWriterT<NetworkDriver>::onNewAckNack(
    const SubmessageAckNack &msg, const GuidPrefix_t &sourceGuidPrefix) {
  INIT_GUARD()
  Lock lock{m_mutex};
  if (!m_is_initialized_) {
    return;
  }

  ReaderProxy *reader = nullptr;
  for (auto &proxy : m_proxies) {
    if (proxy.remoteReaderGuid.prefix == sourceGuidPrefix &&
        proxy.remoteReaderGuid.entityId == msg.readerId) {
      reader = &proxy;
      break;
    }
  }

  if (reader == nullptr) {
#if SFW_VERBOSE && RTPS_GLOBAL_VERBOSE
    SFW_LOG("No proxy found with id: ");
    printEntityId(msg.readerId);
    SFW_LOG(" Dropping acknack.\n");
#endif
    return;
  }

  reader->ackNackCount = msg.count;
  reader->finalFlag = msg.header.finalFlag();
  reader->lastAckNackSequenceNumber = msg.readerSNState.base;

  rtps::SequenceNumber_t nextSN = msg.readerSNState.base;

  // Preemptive ack nack
  if (nextSN.low == 0 && nextSN.high == 0) {
    sendHeartBeat();
    return;
  }

  if (m_history.isEmpty()) {
    // We have never sent anything -> heartbeat
    if (m_history.getLastUsedSequenceNumber() == rtps::SequenceNumber_t{0, 0}) {
      sendHeartBeat();
    } else {
      // No data but we have sent something in the past -> GapStart =
      // readerSNState.base, NextValid = lastUsedSequenceNumber+1
      rtps::SequenceNumber_t nextValid = m_history.getLastUsedSequenceNumber();
      ++nextValid;
      sendGap(*reader, msg.readerSNState.base, nextValid);
    }

    return;
  }

  // Requesting smaller SN than minimum sequence number -> sendGap
  if (msg.readerSNState.base < m_history.getCurrentSeqNumMin()) {
    sendGap(*reader, msg.readerSNState.base, m_history.getCurrentSeqNumMin());
    return;
  }

  SFW_LOG("Received non-preemptive acknack with %lu bits set.\r\n",
          msg.readerSNState.numBits);
  for (uint32_t i = 0; i < msg.readerSNState.numBits &&
                       nextSN <= m_history.getLastUsedSequenceNumber();
       ++i, ++nextSN) {

    if (msg.readerSNState.isSet(i)) {

      SFW_LOG("Looking for change %lu | Bit %lu", nextSN.low, i);
      const rtps::CacheChange *cache = m_history.getChangeBySN(nextSN);

      // We still have the cache, send DATA
      if (cache != nullptr) {
        if (cache->disposeAfterWrite) {
          SFW_LOG("SERVING FROM DISPOSE AFTER WRITE CACHE\r\n");
        }
        sendData(*reader, cache);
      } else {
        SFW_LOG("> Change not found, search for next valid SN %lu \r\n",
                nextSN.low);
        // Cache not found, look for next valid SN
        rtps::SequenceNumber_t gapBegin = nextSN;
        rtps::CacheChange *nextValidChange = nullptr;
        uint32_t j = i + 1;
        for (++nextSN; nextSN <= m_history.getLastUsedSequenceNumber();
             ++nextSN, ++j) {
          nextValidChange = m_history.getChangeBySN(nextSN);
          if (nextValidChange != nullptr) {
            break;
          }
        }
        if (nextValidChange == nullptr) {
          sendGap(*reader, gapBegin, nextSN);
          return;
        } else {
          sendGap(*reader, gapBegin, nextValidChange->sequenceNumber);
        }
        // sendData(nullptr, nextValidChange);
        nextSN = nextValidChange->sequenceNumber;
        --nextSN;
        i = --j;
      }
    }
  }
}

template <class NetworkDriver>
bool rtps::StatefulWriterT<NetworkDriver>::removeFromHistory(
    const SequenceNumber_t &s) {
  Lock lock{m_mutex};
  return m_history.dropChange(s);
}

template <class NetworkDriver>
bool StatefulWriterT<NetworkDriver>::sendData(const ReaderProxy &reader,
                                              const CacheChange *next) {
  INIT_GUARD()
  // TODO smarter packaging, e.g. create a message struct and serialize once.

  PacketInfo info;
  info.srcPort = m_srcPort;
  PayloadBuffer payload;

  MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
  MessageFactory::addSubMessageTimeStamp(payload);

  // Just usable for IPv4
  const LocatorIPv4 &locator = reader.remoteLocator;

  info.destAddr = locator.getIp4AddressBytes();
  info.destPort = (Ip4Port_t)locator.port;

  MessageFactory::addSubMessageData(
      payload, next->data, next->inLineQoS, next->sequenceNumber,
      m_attributes.endpointGuid.entityId, reader.remoteReaderGuid.entityId);
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return false;
  }
  m_transport->sendPacket(info);

  return true;
}

template <class NetworkDriver>
void StatefulWriterT<NetworkDriver>::sendGap(
    const ReaderProxy &reader, const SequenceNumber_t &firstMissing,
    const SequenceNumber_t &nextValid) {
  INIT_GUARD()
  // TODO smarter packaging, e.g. create a message struct and serialize once.

  PacketInfo info;
  info.srcPort = m_srcPort;
  PayloadBuffer payload;

  MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
  MessageFactory::addSubMessageTimeStamp(payload);

  // Just usable for IPv4
  const LocatorIPv4 &locator = reader.remoteLocator;

  info.destAddr = locator.getIp4AddressBytes();
  info.destPort = (Ip4Port_t)locator.port;

  MessageFactory::addSubmessageGap(
      payload, m_attributes.endpointGuid.entityId,
      reader.remoteReaderGuid.entityId, firstMissing, nextValid);
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return;
  }
  m_transport->sendPacket(info);
}

template <class NetworkDriver>
bool StatefulWriterT<NetworkDriver>::sendDataWRMulticast(
    const ReaderProxy &reader, const CacheChange *next) {
  INIT_GUARD()

  if (reader.useMulticast || reader.suppressUnicast == false) {
    PacketInfo info;
    info.srcPort = m_srcPort;
    PayloadBuffer payload;

    MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
    MessageFactory::addSubMessageTimeStamp(payload);

    // Decide whether to use multicast or unicast.
    if (reader.useMulticast) {
      const LocatorIPv4 &locator = reader.remoteMulticastLocator;
      info.destAddr = locator.getIp4AddressBytes();
      info.destPort = (Ip4Port_t)locator.port;
    } else {
      const LocatorIPv4 &locator = reader.remoteLocator;
      info.destAddr = locator.getIp4AddressBytes();
      info.destPort = (Ip4Port_t)locator.port;
    }

    EntityId_t reid;
    if (reader.useMulticast) {
      reid = ENTITYID_UNKNOWN;
    } else {
      reid = reader.remoteReaderGuid.entityId;
    }

    MessageFactory::addSubMessageData(payload, next->data, next->inLineQoS,
                                      next->sequenceNumber,
                                      m_attributes.endpointGuid.entityId, reid);

    info.payload = std::move(payload.bytes);
    if (info.payload.empty()) {
      return false;
    }
    m_transport->sendPacket(info);
  }
  return true;
}

template <class NetworkDriver>
void StatefulWriterT<NetworkDriver>::sendHeartBeatLoop() {
  m_thread_running = true;
  while (m_running) {
    sendHeartBeat();
    dropDisposeAfterWriteChanges();
    bool unconfirmed_changes = false;
    for (auto it : m_proxies) {
      if (it.lastAckNackSequenceNumber < m_nextSequenceNumberToSend) {
        unconfirmed_changes = true;
        break;
      }
    }

    // Temporarily increase HB frequency if there are unconfirmed remote changes
    if (unconfirmed_changes) {
      SFW_LOG("HB SPEEDUP!\r\n");
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Config::SF_WRITER_HB_PERIOD_MS / 4));
    } else {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(Config::SF_WRITER_HB_PERIOD_MS));
    }
  }
  m_thread_running = false;
}

template <class NetworkDriver>
void StatefulWriterT<NetworkDriver>::dropDisposeAfterWriteChanges() {
  SequenceNumber_t oldest_retained;
  while (m_disposeWithDelay.peakFirst(oldest_retained)) {

    CacheChange *change = m_history.getChangeBySN(oldest_retained);
    if (change == nullptr || !change->disposeAfterWrite) {
      // Not in history anymore, drop
      m_disposeWithDelay.moveFirstInto(oldest_retained);
      return;
    }

    if (change->sentTime == CacheChange::TimePoint{}) {
      m_history.dropChange(change->sequenceNumber);
      SequenceNumber_t tmp;
      m_disposeWithDelay.moveFirstInto(tmp);
      continue;
    }

    auto age = std::chrono::steady_clock::now() - change->sentTime;
    if (age > std::chrono::milliseconds(4000)) {
      m_history.dropChange(change->sequenceNumber);
      SFW_LOG("Removing SN %u %u for good\r\n",
              static_cast<unsigned int>(oldest_retained.low),
              static_cast<unsigned int>(oldest_retained.high));
      SequenceNumber_t tmp;
      m_disposeWithDelay.moveFirstInto(tmp);

      continue;
    } else {
      return;
    }
  }
}

template <class NetworkDriver>
void StatefulWriterT<NetworkDriver>::sendHeartBeat() {
  INIT_GUARD()
  if (m_proxies.isEmpty() || !m_is_initialized_) {

    SFW_LOG("Skipping heartbeat. No proxies.\n");
    return;
  }

  for (auto &proxy : m_proxies) {

    PacketInfo info;
    info.srcPort = m_srcPort;
    PayloadBuffer payload;

    SequenceNumber_t firstSN;
    SequenceNumber_t lastSN;

    MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);

    {
      Lock lock{m_mutex};

      if (!m_history.isEmpty()) {
        firstSN = m_history.getCurrentSeqNumMin();
        lastSN = m_history.getCurrentSeqNumMax();

        // Otherwise we may announce changes that have not been sent at least
        // once!
        if (lastSN > m_nextSequenceNumberToSend ||
            lastSN == m_nextSequenceNumberToSend) {
          lastSN = m_nextSequenceNumberToSend;
          --lastSN;
        }

        // Proxy has confirmed all sequence numbers and set final flag
        if ((proxy.lastAckNackSequenceNumber > lastSN) && proxy.finalFlag &&
            proxy.ackNackCount.value > 0) {
          continue;
        }
      } else if (m_history.getLastUsedSequenceNumber() ==
                 SequenceNumber_t{0, 0}) {
        firstSN = SequenceNumber_t{0, 1};
        lastSN = SequenceNumber_t{0, 0};
      } else {
        firstSN = SequenceNumber_t{0, 1};
        lastSN = m_history.getLastUsedSequenceNumber();
      }
    }

    SFW_LOG("Sending HB with SN range [%lu.%lu;%lu.%lu]", firstSN.low, firstSN.high,
            lastSN.low, lastSN.high);

    MessageFactory::addHeartbeat(
        payload, m_attributes.endpointGuid.entityId,
        proxy.remoteReaderGuid.entityId, firstSN, lastSN, m_hbCount);

    info.destAddr = proxy.remoteLocator.getIp4AddressBytes();
    info.destPort = proxy.remoteLocator.port;
    info.payload = std::move(payload.bytes);
    if (info.payload.empty()) {
      continue;
    }
    m_transport->sendPacket(info);
  }
  m_hbCount.value++;
}
