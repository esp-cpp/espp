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

#include "rtps/entities/StatefulReader.hpp"
#include "rtps/communication/EsppTransport.hpp"
#include "rtps/messages/MessageFactory.hpp"
#include "rtps/storages/PayloadBuffer.hpp"
#include "rtps/utils/Diagnostics.hpp"
#include "rtps/utils/Log.hpp"
#include <mutex>

#if SFR_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.hpp"
#define SFR_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define SFR_LOG(...)                                                                               \
  do {                                                                                             \
  } while (0)
#endif

using rtps::Guid_t;
using rtps::GuidPrefix_t;
using rtps::PacketInfo;
using rtps::ReaderCacheChange;
using rtps::SequenceNumber_t;
using rtps::SequenceNumberSet;
using rtps::StatefulReader;
using rtps::SubmessageGap;
using rtps::SubmessageHeartbeat;
using rtps::TopicData;
using rtps::WriterProxy;

StatefulReader::~StatefulReader() {}

bool StatefulReader::init(const TopicData &attributes, EsppTransport &driver) {
  if (!initMutex()) {
    return false;
  }

  m_proxies.clear();
  m_attributes = attributes;
  m_transport = &driver;
  m_srcPort = attributes.unicastLocator.port;
  m_is_initialized_ = true;
  return true;
}

void StatefulReader::newChange(const ReaderCacheChange &cacheChange) {
  if (m_callback_count == 0 || !m_is_initialized_) {
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  for (auto &proxy : m_proxies) {
    if (proxy.remoteWriterGuid == cacheChange.writerGuid) {
      if (proxy.expectedSN == cacheChange.sn) {
        SFR_LOG("Delivering SN {}.{} | GUID {} {} {} {}", (int)cacheChange.sn.high,
                (int)cacheChange.sn.low, cacheChange.writerGuid.prefix.id[0],
                cacheChange.writerGuid.prefix.id[1], cacheChange.writerGuid.prefix.id[2],
                cacheChange.writerGuid.prefix.id[3]);
        executeCallbacks(cacheChange);
        ++proxy.expectedSN;
        SFR_LOG("Done processing SN {}.{}", (int)cacheChange.sn.high, (int)cacheChange.sn.low);
        return;
      } else {
        Diagnostics::StatefulReader::sfr_unexpected_sn++;
        SFR_LOG("Unexpected SN {}.{} != {}.{}, dropping! GUID {} {} {} {}",
                (int)proxy.expectedSN.high, (int)proxy.expectedSN.low, (int)cacheChange.sn.high,
                (int)cacheChange.sn.low, cacheChange.writerGuid.prefix.id[0],
                cacheChange.writerGuid.prefix.id[1], cacheChange.writerGuid.prefix.id[2],
                cacheChange.writerGuid.prefix.id[3]);
      }
    }
  }
}

bool StatefulReader::addNewMatchedWriter(const WriterProxy &newProxy) {
#if SFR_VERBOSE && RTPS_GLOBAL_VERBOSE
  SFR_LOG("New writer added");
#endif
  return m_proxies.add(newProxy);
}

bool StatefulReader::onNewGapMessage(const SubmessageGap &msg, const GuidPrefix_t &remotePrefix) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  if (!m_is_initialized_) {
    return false;
  }
  SFR_LOG("Processing gap message {}.{} {}.{}", (int)msg.gapStart.high,
          (unsigned int)msg.gapStart.low, (int)msg.gapList.base.high,
          (unsigned int)msg.gapList.base.low);

  Guid_t writerProxyGuid;
  writerProxyGuid.prefix = remotePrefix;
  writerProxyGuid.entityId = msg.writerId;
  WriterProxy *writer = getProxy(writerProxyGuid);

  if (writer == nullptr) {

#if SFR_VERBOSE && RTPS_GLOBAL_VERBOSE
    SFR_LOG("Ignore GAP. Couldn't find a matching writer");
#endif
    return false;
  }

  // Case 1: We are still waiting for messages before gapStart
  if (writer->expectedSN < msg.gapStart) {
    PacketInfo info;
    info.srcPort = m_srcPort;
    info.destAddr = writer->remoteLocator.getIp4AddressBytes();
    info.destPort = writer->remoteLocator.port;
    PayloadBuffer payload;
    rtps::MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
    SequenceNumber_t last_valid = msg.gapStart;
    --last_valid;
    auto missing_sns = writer->getMissing(writer->expectedSN, last_valid);
    rtps::MessageFactory::addAckNack(payload, msg.writerId, msg.readerId, missing_sns,
                                     writer->getNextAckNackCount(), false);
    info.payload = std::move(payload.bytes);
    if (info.payload.empty()) {
      return false;
    }
    m_transport->sendPacket(info);
    return true;
  }

  // Case 2: We are expecting a message between [gapStart; gapList.base -1]
  // Advance expectedSN beyond gapList.base
  if (writer->expectedSN < msg.gapList.base) {
    writer->expectedSN = msg.gapList.base;

    // writer->expectedSN++;

    // Advance expectedSN to first unset bit
    for (uint32_t bit = 0; bit < SNS_MAX_NUM_BITS; writer->expectedSN++, bit++) {
      if (!msg.gapList.isSet(bit)) {
        break;
      }
    }

    return true;

  } else {

    // Case 3: We are expecting a sequence number beyond gap list base,
    // check if we need to update expectedSN
    auto i = msg.gapList.base;
    for (uint32_t bit = 0; bit < SNS_MAX_NUM_BITS; i++, bit++) {
      if (i < writer->expectedSN) {
        continue;
      }

      if (msg.gapList.isSet(bit)) {
        writer->expectedSN++;
      } else {
        PacketInfo info;
        info.srcPort = m_srcPort;
        info.destAddr = writer->remoteLocator.getIp4AddressBytes();
        info.destPort = writer->remoteLocator.port;
        PayloadBuffer payload;
        rtps::MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
        SequenceNumberSet set;
        set.base = writer->expectedSN;
        set.numBits = 1;
        set.bitMap[0] = set.bitMap[0] |= uint32_t{1} << 31;
        rtps::MessageFactory::addAckNack(payload, msg.writerId, msg.readerId, set,
                                         writer->getNextAckNackCount(), false);
        info.payload = std::move(payload.bytes);
        if (info.payload.empty()) {
          return false;
        }
        m_transport->sendPacket(info);

        return true;
      }
    }

    return false;
  }
}

bool StatefulReader::onNewHeartbeat(const SubmessageHeartbeat &msg,
                                    const GuidPrefix_t &sourceGuidPrefix) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  if (!m_is_initialized_) {
    return false;
  }
  PacketInfo info;
  info.srcPort = m_srcPort;
  PayloadBuffer payload;

  Guid_t writerProxyGuid;
  writerProxyGuid.prefix = sourceGuidPrefix;
  writerProxyGuid.entityId = msg.writerId;
  WriterProxy *writer = getProxy(writerProxyGuid);

  if (writer == nullptr) {

#if SFR_VERBOSE && RTPS_GLOBAL_VERBOSE
    SFR_LOG("Ignore heartbeat. Couldn't find a matching writer");
#endif
    return false;
  }

  if (writer->expectedSN < msg.firstSN) {
    SFR_LOG("expectedSN < firstSN, advancing expectedSN");
    writer->expectedSN = msg.firstSN;
  }

  writer->hbCount.value = msg.count.value;
  info.destAddr = writer->remoteLocator.getIp4AddressBytes();
  info.destPort = writer->remoteLocator.port;
  rtps::MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
  auto missing_sns = writer->getMissing(msg.firstSN, msg.lastSN);
  bool final_flag = (missing_sns.numBits == 0);
  rtps::MessageFactory::addAckNack(payload, msg.writerId, msg.readerId, missing_sns,
                                   writer->getNextAckNackCount(), final_flag);

  SFR_LOG("Sending acknack base {} bits {}.", (int)missing_sns.base.low, (int)missing_sns.numBits);
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return false;
  }
  m_transport->sendPacket(info);
  return true;
}

bool StatefulReader::sendPreemptiveAckNack(const WriterProxy &writer) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  if (!m_is_initialized_) {
    return false;
  }

  PacketInfo info;
  info.srcPort = m_attributes.unicastLocator.port;
  info.destAddr = writer.remoteLocator.getIp4AddressBytes();
  info.destPort = writer.remoteLocator.port;
  PayloadBuffer payload;
  rtps::MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
  SequenceNumberSet number_set;
  number_set.base.high = 0;
  number_set.base.low = 0;
  number_set.numBits = 0;
  rtps::MessageFactory::addAckNack(payload, writer.remoteWriterGuid.entityId,
                                   m_attributes.endpointGuid.entityId, number_set, Count_t{1},
                                   false);

  SFR_LOG("Sending preemptive acknack.");
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return false;
  }
  m_transport->sendPacket(info);
  return true;
}
