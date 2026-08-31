/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Modifications Copyright (c) 2026 ATDev
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

#include "rtps/entities/StatefulWriter.hpp"
#include "rtps/communication/EsppTransport.hpp"
#include "rtps/messages/MessageFactory.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/storages/PayloadBuffer.hpp"
#include "rtps/utils/Diagnostics.hpp"
#include "rtps/utils/Log.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

using rtps::CacheChange;
using rtps::GuidPrefix_t;
using rtps::ReaderProxy;
using rtps::SequenceNumber_t;
using rtps::StatefulWriter;
using rtps::SubmessageAckNack;

#if SFW_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.hpp"
#define SFW_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define SFW_LOG(...)                                                                               \
  do {                                                                                             \
  } while (0)
#endif

StatefulWriter::~StatefulWriter() = default;

bool StatefulWriter::init(TopicData attributes, TopicKind_t topicKind, EsppTransport &driver,
                          bool enfUnicast) {
  // Take m_mutex across the FULL (re)initialization: the protocol scheduler's
  // heartbeatTick() and a stale generation-guarded progress() job read
  // m_is_initialized_ / m_nextHeartbeat / m_proxies / m_history under this
  // lock, and a pooled writer slot can be re-init()ed while either is running
  // on another thread - unlocked writes here would be a data race exposing
  // partially initialized state.
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  m_attributes = attributes;

  m_srcPort = attributes.unicastLocator.port;
  m_enforceUnicast = enfUnicast;
  m_topicKind = topicKind;

  m_nextSequenceNumberToSend = {0, 1};
  m_proxies.clear();

  m_transport = &driver;
  m_history.clear();
  m_hbCount = {1};

  // Reused pooled slot: this is a NEW logical writer, so its public drop
  // counter must not inherit the previous endpoint's total (the facade
  // attributes publish()-time overflow warnings to it).
  m_history_drops_ = 0;

  // Thread already exists, do not create new one (reusing slot case)
  m_is_initialized_ = true;

  // Heartbeats are driven by the Domain's protocol scheduler via
  // heartbeatTick(); no per-writer heartbeat thread. Arm the first evaluation.
  m_nextHeartbeat = std::chrono::steady_clock::now();

  return true;
}

void StatefulWriter::reset() {
  // Clear the init flag under m_mutex so it synchronizes with progress() /
  // newChange() (which read it under the same lock): an in-flight progress()
  // completes before reset() proceeds, and any later job sees !initialized.
  // Bump the generation so an already-accepted guaranteed job cannot run
  // against this slot once it is reused for another endpoint.
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  m_is_initialized_ = false;
  ++m_generation_;
  // TODO
}

const rtps::CacheChange *
StatefulWriter::newChange(ChangeKind_t kind, const uint8_t *data, DataSize_t size, bool inLineQoS,
                          bool markDisposedAfterWrite, bool hasRelatedSampleIdentity,
                          const rpc::SampleIdentity &relatedSampleIdentity) {
  INIT_GUARD()
  if (isIrrelevant(kind)) {
    return nullptr;
  }

  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (!m_is_initialized_) {
    return nullptr;
  }

  // A full history may drop its oldest change on the next addChange: the static
  // ring always does, and the dynamic (host) ring does too when it hits its
  // 16-bit index ceiling and grow() refuses to resize. When a drop happens the
  // send cursor may point at a sequence number that no longer exists, which
  // would stall progress() permanently; advance it past the drop. When the
  // dynamic ring instead grows and RETAINS the oldest (the common host case) the
  // minimum is unchanged, so the cursor stays put and the retained change is
  // still sent. Distinguish the two by whether the history minimum advanced
  // across the add - a copy, not a reference, because addChange may overwrite
  // the underlying slot.
  const bool wasFull = m_history.isFull();
  const SequenceNumber_t minBefore = m_history.getCurrentSeqNumMin();

  auto *result = m_history.addChange(data, size, inLineQoS, markDisposedAfterWrite,
                                     hasRelatedSampleIdentity, relatedSampleIdentity);

  if (wasFull) {
    const SequenceNumber_t minAfter = m_history.getCurrentSeqNumMin();
    if (minBefore < minAfter && m_nextSequenceNumberToSend < minAfter) {
      m_nextSequenceNumberToSend = minAfter; // Skip past the dropped change
      // Count the loss (an UNSENT change was overwritten): per-writer for the
      // facade's publish()-time warning, process-wide for Diagnostics.
      ++m_history_drops_;
      ++Diagnostics::Writer::history_overwrite_drops;
      SFW_LOG("History full, dropped oldest {}.", this->m_attributes.topicName);
    }
  }
  if (m_transport != nullptr) {
    // Run the send asynchronously on the transport's worker pool (never inline
    // under the caller's locks), matching the previous ThreadPool semantics.
    // Guaranteed + banded: a bounded-queue rejection must not strand unsent
    // samples (a lone best-effort DATA has no recovery path), and a
    // prioritized endpoint's outbound work runs at ITS band end-to-end.
    m_transport->submitGuaranteedDrain(
        this, [this, gen = currentGeneration()]() { progressIfCurrent(gen); }, m_attributes.band);
  }
  // Piggyback: pull the next heartbeat evaluation forward so a reliable
  // publish is followed promptly by a HEARTBEAT instead of waiting out the
  // period, and nudge the protocol scheduler.
  m_nextHeartbeat = std::chrono::steady_clock::now();
  if (m_protocolNudge) {
    m_protocolNudge();
  }

  SFW_LOG("Adding new data.");

  return result;
}

void StatefulWriter::progress() {
  INIT_GUARD()
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  // A guaranteed progress() job may still be parked/queued when this writer is
  // deleted. reset() clears m_is_initialized_ under m_mutex, so a job that runs
  // after deletion no-ops here instead of sending the (now reset) history or
  // touching an already-released dedicated port. Mirrors newChange()'s guard.
  if (!m_is_initialized_) {
    return;
  }
  // Skip any hole the cursor points at: a change can be dropped WITHOUT the
  // cursor advancing past it - e.g. an unsent dispose-after-write removed by
  // dropDisposeAfterWriteChanges() when its endpoint was deleted before the
  // send cursor reached it (rapid endpoint churn). The SN is gone from
  // history, so it can never be sent; without advancing, the cursor would be
  // stuck and every LATER change (for the SEDP writer: every subsequent
  // endpoint announcement of this participant) never transmitted. Jump to the
  // history minimum when the cursor fell behind it, and step past mid-history
  // holes until a live change (or the end) is reached - in THIS poke, so one
  // poke cannot be swallowed by a run of consecutive holes. Readers are told
  // of the skip by the normal GAP/heartbeat machinery (same recovery as the
  // history-full drop in newChange()).
  CacheChange *next = m_history.getChangeBySN(m_nextSequenceNumberToSend);
  if (next == nullptr && !m_history.isEmpty()) {
    const SequenceNumber_t minSN = m_history.getCurrentSeqNumMin();
    if (m_nextSequenceNumberToSend < minSN) {
      SFW_LOG("Cursor fell behind history; jumping to SN ({},{})", minSN.high, minSN.low);
      m_nextSequenceNumberToSend = minSN;
      next = m_history.getChangeBySN(m_nextSequenceNumberToSend);
    }
    while (next == nullptr && m_nextSequenceNumberToSend < m_history.getCurrentSeqNumMax()) {
      ++m_nextSequenceNumberToSend; // mid-history hole: step past it
      next = m_history.getChangeBySN(m_nextSequenceNumberToSend);
    }
  }
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

    SFW_LOG("Sending data with SN {}.{}", (int)m_nextSequenceNumberToSend.low,
            (int)m_nextSequenceNumberToSend.high);

    /*
     * Use case: deletion of local endpoints
     * -> send Data Message with Disposed Flag set
     * -> Set respective SEDP CacheChange as NOT_ALIVE_DISPOSED after
     * transmission to proxies
     * -> onAckNack will send Gap Messages to skip deleted local endpoints
     * during SEDP
     */
    if (next->disposeAfterWrite) {
      SFW_LOG("Dispose after write msg sent to {} proxies", (int)i);
      next->sentTime = std::chrono::steady_clock::now();
      if (!m_disposeWithDelay.copyElementIntoBuffer(next->sequenceNumber)) {
        SFW_LOG("Failed to enqueue dispose after write!");
        m_history.dropChange(next->sequenceNumber);
      } else {
        SFW_LOG("Delayed dispose scheduled for sn {} {}", (int)next->sequenceNumber.high,
                (int)next->sequenceNumber.low);
      }
    }

    ++m_nextSequenceNumberToSend;
    SFW_LOG("HB from progress");
    sendHeartBeat();

    // Drain re-arm (see EsppTransport::submitGuaranteedDrain): pokes park
    // with a pending-count of at most ONE, so this run must resubmit itself
    // while unsent samples remain - each admitted run sends one sample and
    // re-arms until the cursor catches up with the history, keeping the
    // parked debt per writer bounded at one (m_mutex is held, so the
    // cursor/history read is stable).
    if (!m_history.isEmpty() && m_nextSequenceNumberToSend <= m_history.getCurrentSeqNumMax() &&
        m_transport != nullptr) {
      m_transport->submitGuaranteedDrain(
          this, [this, gen = currentGeneration()]() { progressIfCurrent(gen); }, m_attributes.band);
    }
  } else {
    SFW_LOG("Couldn't get a CacheChange with SN ({},{})", m_nextSequenceNumberToSend.high,
            m_nextSequenceNumberToSend.low);
  }
}

void StatefulWriter::setAllChangesToUnsent() {
  INIT_GUARD()
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  m_nextSequenceNumberToSend = m_history.getCurrentSeqNumMin();

  if (m_transport != nullptr) {
    // Run the send asynchronously on the transport's worker pool (never inline
    // under the caller's locks), matching the previous ThreadPool semantics.
    // Guaranteed + banded: a bounded-queue rejection must not strand unsent
    // samples (a lone best-effort DATA has no recovery path), and a
    // prioritized endpoint's outbound work runs at ITS band end-to-end.
    m_transport->submitGuaranteedDrain(
        this, [this, gen = currentGeneration()]() { progressIfCurrent(gen); }, m_attributes.band);
  }
  // Piggyback: pull the next heartbeat evaluation forward so a reliable
  // publish is followed promptly by a HEARTBEAT instead of waiting out the
  // period, and nudge the protocol scheduler.
  m_nextHeartbeat = std::chrono::steady_clock::now();
  if (m_protocolNudge) {
    m_protocolNudge();
  }
}

void StatefulWriter::onNewAckNack(const SubmessageAckNack &msg,
                                  const GuidPrefix_t &sourceGuidPrefix) {
  INIT_GUARD()
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (!m_is_initialized_) {
    return;
  }

  auto proxy_it = std::find_if(m_proxies.begin(), m_proxies.end(), [&](const auto &proxy) {
    return proxy.remoteReaderGuid.prefix == sourceGuidPrefix &&
           proxy.remoteReaderGuid.entityId == msg.readerId;
  });
  ReaderProxy *reader = (proxy_it != m_proxies.end()) ? &(*proxy_it) : nullptr;

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
    SFW_LOG("Received preemptive acknack, sending heartbeat.");
    sendHeartBeat();
    return;
  }

  if (m_history.isEmpty()) {
    // We have never sent anything. Do not immediately respond with another
    // heartbeat here, otherwise reader/writer can get stuck in HB<->ACKNACK
    // ping-pong. Periodic heartbeat still handles liveliness.
    if (m_history.getLastUsedSequenceNumber() == rtps::SequenceNumber_t{0, 0}) {
      SFW_LOG("Ignoring acknack while history is empty and no samples were sent yet.");
      return;
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

  SFW_LOG("Received non-preemptive acknack with {} bits set.", msg.readerSNState.numBits);
  for (uint32_t i = 0;
       i < msg.readerSNState.numBits && nextSN <= m_history.getLastUsedSequenceNumber();
       ++i, ++nextSN) {

    if (msg.readerSNState.isSet(i)) {

      SFW_LOG("Looking for change {} | Bit {}", nextSN.low, i);
      const rtps::CacheChange *cache = m_history.getChangeBySN(nextSN);

      // We still have the cache, send DATA
      if (cache != nullptr) {
        if (cache->disposeAfterWrite) {
          SFW_LOG("Serving from dispose-after-write cache");
        }
        sendData(*reader, cache);
      } else {
        SFW_LOG("> Change not found, search for next valid SN {}", nextSN.low);
        // Cache not found, look for next valid SN
        rtps::SequenceNumber_t gapBegin = nextSN;
        rtps::CacheChange *nextValidChange = nullptr;
        uint32_t j = i + 1;
        for (++nextSN; nextSN <= m_history.getLastUsedSequenceNumber(); ++nextSN, ++j) {
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

bool rtps::StatefulWriter::removeFromHistory(const SequenceNumber_t &s) {
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  return m_history.dropChange(s);
}

bool StatefulWriter::sendData(const ReaderProxy &reader, const CacheChange *next) {
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

  if (next->hasRelatedSampleIdentity) {
    // ROS 2 service request/reply: carry related_sample_identity as inline QoS.
    // Such a sample must fit one DATA submessage - DATA_FRAG carries no inline
    // QoS, so fragmenting it would drop the correlation and the caller would
    // time out. The RPC facade rejects payloads above MAX_UNFRAGMENTED_RPC_PAYLOAD;
    // guard here too rather than emit an uncorrelated fragment.
    if (next->data.spaceUsed() > MAX_UNFRAGMENTED_RPC_PAYLOAD) {
      return false;
    }
    MessageFactory::addSubMessageDataWithRelatedSampleIdentity(
        payload, next->data, next->relatedSampleIdentity, next->sequenceNumber,
        m_attributes.endpointGuid.entityId, reader.remoteReaderGuid.entityId);
  } else {
#ifdef RTPS_ENABLE_FRAGMENTATION
    if (next->data.spaceUsed() > MAX_UNFRAGMENTED_PAYLOAD) {
      return sendSampleFragmented(info.destAddr, info.destPort, reader.remoteReaderGuid.entityId,
                                  next);
    }
#endif
    MessageFactory::addSubMessageData(payload, next->data, next->inLineQoS, next->sequenceNumber,
                                      m_attributes.endpointGuid.entityId,
                                      reader.remoteReaderGuid.entityId);
  }
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return false;
  }
  m_transport->sendPacket(info);

  return true;
}

#ifdef RTPS_ENABLE_FRAGMENTATION
bool StatefulWriter::sendSampleFragmented(const Ip4AddressBytes &destAddr, Ip4Port_t destPort,
                                          const EntityId_t &readerId, const CacheChange *next) {
  INIT_GUARD()
  const uint8_t *sampleData = next->data.bytes.data();
  const uint32_t sampleSize = next->data.spaceUsed();
  const uint16_t fragmentSize = m_fragmentSize > MAX_FRAGMENT_SIZE
                                    ? static_cast<uint16_t>(MAX_FRAGMENT_SIZE)
                                    : m_fragmentSize;
  if (fragmentSize == 0) {
    return false;
  }
  const uint32_t numFragments = (sampleSize + fragmentSize - 1) / fragmentSize;
  for (uint32_t f = 0; f < numFragments; ++f) {
    const uint32_t offset = f * fragmentSize;
    const uint16_t fragLen = static_cast<uint16_t>(
        (sampleSize - offset) < fragmentSize ? (sampleSize - offset) : fragmentSize);

    PacketInfo info;
    info.srcPort = m_srcPort;
    info.destAddr = destAddr;
    info.destPort = destPort;
    PayloadBuffer payload;
    MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
    MessageFactory::addSubMessageTimeStamp(payload);
    MessageFactory::addSubMessageDataFrag(payload, sampleData + offset, fragLen, f + 1, 1,
                                          fragmentSize, sampleSize, next->sequenceNumber,
                                          m_attributes.endpointGuid.entityId, readerId);
    info.payload = std::move(payload.bytes);
    if (info.payload.empty()) {
      return false;
    }
    m_transport->sendPacket(info);
  }
  return true;
}
#endif

void StatefulWriter::sendGap(const ReaderProxy &reader, const SequenceNumber_t &firstMissing,
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

  MessageFactory::addSubmessageGap(payload, m_attributes.endpointGuid.entityId,
                                   reader.remoteReaderGuid.entityId, firstMissing, nextValid);
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return;
  }
  m_transport->sendPacket(info);
}

bool StatefulWriter::sendDataWRMulticast(const ReaderProxy &reader, const CacheChange *next) {
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

#ifdef RTPS_ENABLE_FRAGMENTATION
    if (next->data.spaceUsed() > MAX_UNFRAGMENTED_PAYLOAD) {
      return sendSampleFragmented(info.destAddr, info.destPort, reid, next);
    }
#endif

    if (next->hasRelatedSampleIdentity) {
      // ROS 2 service request/reply: carry related_sample_identity as inline QoS.
      MessageFactory::addSubMessageDataWithRelatedSampleIdentity(
          payload, next->data, next->relatedSampleIdentity, next->sequenceNumber,
          m_attributes.endpointGuid.entityId, reid);
    } else {
      MessageFactory::addSubMessageData(payload, next->data, next->inLineQoS, next->sequenceNumber,
                                        m_attributes.endpointGuid.entityId, reid);
    }

    info.payload = std::move(payload.bytes);
    if (info.payload.empty()) {
      return false;
    }
    m_transport->sendPacket(info);
  }
  return true;
}

std::chrono::steady_clock::time_point
StatefulWriter::heartbeatTick(std::chrono::steady_clock::time_point now) {
  // Hold m_mutex across the WHOLE tick: sendHeartBeat() locks it, but the
  // unconfirmed-changes scan below also iterates m_proxies, which the SEDP
  // receive workers mutate under m_mutex - the scan must not run unlocked
  // (m_mutex is recursive, so the nested guards stay harmless).
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (!m_is_initialized_) {
    // Not ticking: report a far-future deadline so the scheduler ignores us.
    return now + std::chrono::hours(24);
  }
  if (now < m_nextHeartbeat) {
    return m_nextHeartbeat;
  }
  SFW_LOG("HB from tick");
  sendHeartBeat();
  dropDisposeAfterWriteChanges();
  const auto pending_ack_proxy =
      std::find_if(m_proxies.begin(), m_proxies.end(), [&](const auto &proxy) {
        return proxy.lastAckNackSequenceNumber < m_nextSequenceNumberToSend;
      });
  const bool unconfirmed_changes = pending_ack_proxy != m_proxies.end();

  // Same cadence as the historical per-writer loop: temporarily increase the
  // HB frequency while there are unconfirmed remote changes.
  if (unconfirmed_changes) {
    SFW_LOG("HB speedup");
    m_nextHeartbeat = now + std::chrono::milliseconds(Config::SF_WRITER_HB_PERIOD_MS / 4);
  } else {
    m_nextHeartbeat = now + std::chrono::milliseconds(Config::SF_WRITER_HB_PERIOD_MS);
  }
  return m_nextHeartbeat;
}

void StatefulWriter::dropDisposeAfterWriteChanges() {
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
      SFW_LOG("Removing SN {} {} for good", static_cast<unsigned int>(oldest_retained.low),
              static_cast<unsigned int>(oldest_retained.high));
      SequenceNumber_t tmp;
      m_disposeWithDelay.moveFirstInto(tmp);

      continue;
    } else {
      return;
    }
  }
}

void StatefulWriter::sendHeartBeat() {
  INIT_GUARD()
  // Hold m_mutex across the WHOLE proxy iteration: matched-reader proxies are
  // added/removed under m_mutex from the SEDP receive workers (endpoint
  // (un)announcements), and iterating the pool unlocked from the protocol
  // task races those mutations - observed as a SIGSEGV in this loop during
  // endpoint churn. m_mutex is recursive, so the pre-existing inner history
  // guard below stays harmless.
  std::lock_guard<std::recursive_mutex> proxies_lock(m_mutex);
  if (m_proxies.isEmpty() || !m_is_initialized_) {

    SFW_LOG("Skipping heartbeat. No proxies.");
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
      std::lock_guard<std::recursive_mutex> lock(m_mutex);

      if (!m_history.isEmpty()) {
        firstSN = m_history.getCurrentSeqNumMin();
        lastSN = m_history.getCurrentSeqNumMax();

        // Otherwise we may announce changes that have not been sent at least
        // once!
        if (lastSN > m_nextSequenceNumberToSend || lastSN == m_nextSequenceNumberToSend) {
          lastSN = m_nextSequenceNumberToSend;
          --lastSN;
        }

        // Proxy has confirmed all sequence numbers and set final flag
        if ((proxy.lastAckNackSequenceNumber > lastSN) && proxy.finalFlag &&
            proxy.ackNackCount.value > 0) {
          SFW_LOG("Skipping heartbeat for proxy, all changes confirmed. lastSN {}.{}, lastAckNack "
                  "{}.{}",
                  (int)lastSN.low, (int)lastSN.high, (int)proxy.lastAckNackSequenceNumber.low,
                  (int)proxy.lastAckNackSequenceNumber.high);
          continue;
        }
      } else if (m_history.getLastUsedSequenceNumber() == SequenceNumber_t{0, 0}) {
        if ((proxy.lastAckNackSequenceNumber > m_history.getLastUsedSequenceNumber()) &&
            proxy.finalFlag && proxy.ackNackCount.value > 0) {
          SFW_LOG("Skipping heartbeat for proxy, all changes confirmed. lastUsedSN {}.{}, "
                  "lastAckNack {}.{}",
                  (int)m_history.getLastUsedSequenceNumber().low,
                  (int)m_history.getLastUsedSequenceNumber().high,
                  (int)proxy.lastAckNackSequenceNumber.low,
                  (int)proxy.lastAckNackSequenceNumber.high);
          continue;
        }
        firstSN = SequenceNumber_t{0, 1};
        lastSN = SequenceNumber_t{0, 0};
      } else {
        firstSN = SequenceNumber_t{0, 1};
        lastSN = m_history.getLastUsedSequenceNumber();
      }
    }

    SFW_LOG("Sending HB with SN range [{}.{};{}.{}]", firstSN.low, firstSN.high, lastSN.low,
            lastSN.high);

    MessageFactory::addHeartbeat(payload, m_attributes.endpointGuid.entityId,
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
