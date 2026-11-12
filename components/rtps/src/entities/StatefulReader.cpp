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
#include <algorithm>
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
  // Serialize the whole delivery (claim + callbacks) on m_delivery_mutex, and
  // hold m_proxies_mutex only for the expectedSN claim - NOT across the user
  // callbacks. Invoking user code under the proxies mutex created a lock-order
  // cycle: a callback calling back into the facade (e.g. add_writer/publish)
  // reaches the facade/SEDP/participant locks, while the SEDP receive path
  // takes those locks and then this reader's proxies mutex
  // (addNewMatchedWriter). The delivery mutex is a leaf acquired first, so the
  // strict in-order, one-callback-at-a-time semantics are preserved while the
  // proxies mutex stays off every user-callback stack.
  std::lock_guard<std::mutex> delivery(m_delivery_mutex);
  bool deliver = false;
  {
    std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
    auto matches_writer = [&](const WriterProxy &proxy) {
      return proxy.remoteWriterGuid == cacheChange.writerGuid;
    };
    auto it = std::find_if(m_proxies.begin(), m_proxies.end(), matches_writer);
    if (it != m_proxies.end()) {
      WriterProxy &proxy = *it;
      if (proxy.lastAnnouncedSN < cacheChange.sn) {
        proxy.lastAnnouncedSN = cacheChange.sn; // a DATA proves the number exists
      }
      if (proxy.expectedSN == cacheChange.sn) {
        // Claim the SN under the proxies mutex; the callbacks run below,
        // still serialized by m_delivery_mutex so delivery order matches
        // claim order.
        ++proxy.expectedSN;
        deliver = true;
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
  if (deliver) {
    SFR_LOG("Delivering SN {}.{} | GUID {} {} {} {}", (int)cacheChange.sn.high,
            (int)cacheChange.sn.low, cacheChange.writerGuid.prefix.id[0],
            cacheChange.writerGuid.prefix.id[1], cacheChange.writerGuid.prefix.id[2],
            cacheChange.writerGuid.prefix.id[3]);
    executeCallbacks(cacheChange);
    SFR_LOG("Done processing SN {}.{}", (int)cacheChange.sn.high, (int)cacheChange.sn.low);
  }
}

bool StatefulReader::addNewMatchedWriter(const WriterProxy &newProxy) {
#if SFR_VERBOSE && RTPS_GLOBAL_VERBOSE
  SFR_LOG("New writer added");
#endif
  // Guard the pool mutation: newChange()/onNewHeartbeat() iterate m_proxies
  // under m_proxies_mutex on the receive workers, and an unlocked add races
  // them (endpoint (re)announcements arrive on a different worker).
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  return m_proxies.add(newProxy);
}

namespace {
// Applies the GAP's advance of expectedSN only as far as the writer has
// announced (see onNewGapMessage), on every return path.
struct CapExpectedSN {
  rtps::WriterProxy *writer;
  rtps::SequenceNumber_t limit;
  ~CapExpectedSN() {
    if (limit < writer->expectedSN) {
      writer->expectedSN = limit;
    }
  }
};
} // namespace

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

  // A GAP names sequence numbers the writer will never send - but only ones
  // it has shown to exist (RTPS 2.5 8.4.9.3.2 lists them as "sent"). Fast DDS
  // sends a GAP for the NEXT number of its SEDP writer after an endpoint
  // deletion (a hole where the deleted endpoint's announcement stood), then
  // writes that very number for the next endpoint it announces: applied as
  // sent, the GAP skipped the announcement and every one after it, so a peer
  // that churned its publishers (the interop matrix's churn case against
  // rmw_fastrtps_cpp) was never matched again. A GAP wholly above the highest
  // number the writer has announced (HEARTBEAT lastSN, or a DATA) is ignored
  // - the next HEARTBEAT decides - and one that reaches past it is applied
  // only up to it. Regression test: pc/tests/rtps_gap_before_data.cpp.
  if (writer->lastAnnouncedSN < msg.gapStart) {
    SFR_LOG("Ignoring a GAP from {}.{} beyond the writer's last announced {}.{}",
            (int)msg.gapStart.high, (int)msg.gapStart.low, (int)writer->lastAnnouncedSN.high,
            (int)writer->lastAnnouncedSN.low);
    return true;
  }
  SequenceNumber_t announcedNext = writer->lastAnnouncedSN;
  ++announcedNext;
  const CapExpectedSN cap{writer, announcedNext};

  // Case 1: We are still waiting for messages before gapStart
  if (writer->expectedSN < msg.gapStart) {
    PacketInfo info;
    info.srcPort = m_srcPort;
    info.destAddr = writer->remoteLocator.getIp4AddressBytes();
    info.destPort = writer->remoteLocator.port;
    PayloadBuffer payload;
    rtps::MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
    GuidPrefix_t dst = writer->remoteWriterGuid.prefix;
    rtps::MessageFactory::addSubMessageInfoDST(payload, dst);
    SequenceNumber_t last_valid = msg.gapStart;
    --last_valid;
    auto missing_sns = writer->getMissing(writer->expectedSN, last_valid);
    rtps::MessageFactory::addAckNack(payload, msg.writerId, m_attributes.endpointGuid.entityId,
                                     missing_sns, writer->getNextAckNackCount(), false);
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
        GuidPrefix_t dst = writer->remoteWriterGuid.prefix;
        rtps::MessageFactory::addSubMessageInfoDST(payload, dst);
        SequenceNumberSet set;
        set.base = writer->expectedSN;
        set.numBits = 1;
        set.bitMap[0] = set.bitMap[0] |= uint32_t{1} << 31;
        rtps::MessageFactory::addAckNack(payload, msg.writerId, m_attributes.endpointGuid.entityId,
                                         set, writer->getNextAckNackCount(), false);
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

  // RTPS 2.5 8.3.7.5.3: a HEARTBEAT is valid only if firstSN > 0 and
  // lastSN >= firstSN - 1. Checked here because the next line trusts firstSN:
  // one accepted with garbage in it (a datagram corrupted in flight, with
  // lwIP's UDP checksum verification off, as it is by default in ESP-IDF)
  // moved expectedSN far ahead and every real sample after that was "old":
  // the subscription stalled for the writer's lifetime. Seen by the QEMU
  // interop matrix's corruption case.
  {
    // Both numbers are untrusted: test firstSN > 0 first, then compare lastSN
    // against firstSN - 1 (a decrement that cannot underflow once firstSN is
    // positive) rather than firstSN against lastSN + 1, which would overflow
    // the signed high word for a lastSN at the top of the range.
    const SequenceNumber_t zero{0, 0};
    bool valid = zero < msg.firstSN;
    if (valid) {
      SequenceNumber_t firstMinusOne = msg.firstSN;
      --firstMinusOne;
      valid = !(msg.lastSN < firstMinusOne);
    }
    if (!valid) {
      SFR_LOG("Ignoring an invalid heartbeat: firstSN {}.{} lastSN {}.{}", (int)msg.firstSN.high,
              (int)msg.firstSN.low, (int)msg.lastSN.high, (int)msg.lastSN.low);
      return false;
    }
  }

  if (writer->lastAnnouncedSN < msg.lastSN) {
    writer->lastAnnouncedSN = msg.lastSN;
  }
  if (writer->expectedSN < msg.firstSN) {
    SFR_LOG("expectedSN < firstSN, advancing expectedSN");
    writer->expectedSN = msg.firstSN;
  }

  writer->hbCount.value = msg.count.value;
  info.destAddr = writer->remoteLocator.getIp4AddressBytes();
  info.destPort = writer->remoteLocator.port;
  rtps::MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
  // RTPS 2.5 8.3.7.1: an ACKNACK should be preceded by INFO_DST. CycloneDDS
  // resolves the writer by (destination prefix, writerId); without INFO_DST the
  // prefix is null and the ACKNACK never reaches the writer's reader table.
  GuidPrefix_t dst = writer->remoteWriterGuid.prefix;
  rtps::MessageFactory::addSubMessageInfoDST(payload, dst);
  auto missing_sns = writer->getMissing(msg.firstSN, msg.lastSN);
  bool final_flag = (missing_sns.numBits == 0);
  rtps::MessageFactory::addAckNack(payload, msg.writerId, m_attributes.endpointGuid.entityId,
                                   missing_sns, writer->getNextAckNackCount(), final_flag);

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
  GuidPrefix_t dst = writer.remoteWriterGuid.prefix;
  rtps::MessageFactory::addSubMessageInfoDST(payload, dst);
  // "Nothing received yet, send from the start": base 1 is the lowest valid
  // sequence number. CycloneDDS validates the base and discards a base of 0
  // (it patches that only for eProsima and RTI vendor ids); FastDDS accepts
  // either.
  SequenceNumberSet number_set;
  number_set.base.high = 0;
  number_set.base.low = 1;
  number_set.numBits = 0;
  // The count comes from the stored proxy's counter, so the regular ACKNACKs
  // that follow (getNextAckNackCount(), which starts at 1) are numbered after
  // it: a Fast DDS writer discards an ACKNACK whose count is not above the
  // last one it saw, and with a fixed 1 here the reader's first repair request
  // was that discarded duplicate (the QEMU soak's 30 KB sample never arrived
  // from a Fast DDS publisher that was gone before the next heartbeat round).
  // The proxy is in m_proxies by the time the SEDP agent calls this; 1 is only
  // the fallback for a caller that passes a proxy that is not stored.
  Count_t count{1};
  if (WriterProxy *stored = getProxy(writer.remoteWriterGuid); stored != nullptr) {
    count = stored->getNextAckNackCount();
  }
  rtps::MessageFactory::addAckNack(payload, writer.remoteWriterGuid.entityId,
                                   m_attributes.endpointGuid.entityId, number_set, count, false);

  SFR_LOG("Sending preemptive acknack.");
  info.payload = std::move(payload.bytes);
  if (info.payload.empty()) {
    return false;
  }
  m_transport->sendPacket(info);
  return true;
}
