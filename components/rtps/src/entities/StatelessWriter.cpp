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

#include "rtps/entities/StatelessWriter.hpp"

#include <rtps/entities/ReaderProxy.hpp>
#include <rtps/entities/Writer.hpp>

#include "rtps/communication/EsppTransport.hpp"
#include "rtps/communication/PacketInfo.hpp"
#include "rtps/messages/MessageFactory.hpp"
#include "rtps/storages/PayloadBuffer.hpp"
#include "rtps/utils/Log.hpp"
#include "rtps/utils/udpUtils.hpp"
#include <mutex>

using rtps::CacheChange;
using rtps::GuidPrefix_t;
using rtps::SequenceNumber_t;
using rtps::StatelessWriter;
using rtps::SubmessageAckNack;

#if SLW_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.hpp"
#define SLW_LOG(...) logger_.warn(__VA_ARGS__)
#else
#define SLW_LOG(...)                                                                               \
  do {                                                                                             \
  } while (0)
#endif

StatelessWriter::~StatelessWriter() {
  //  if(sys_mutex_valid(&m_mutex)){
  //    sys_mutex_free(&m_mutex);
  //  }
}

bool StatelessWriter::init(TopicData attributes, TopicKind_t topicKind, EsppTransport &driver,
                           bool enfUnicast) {

  m_attributes = attributes;

  m_srcPort = attributes.unicastLocator.port;
  m_enforceUnicast = enfUnicast;

  m_topicKind = topicKind;
  m_nextSequenceNumberToSend = {0, 1};
  m_is_initialized_ = true;

  m_proxies.clear();
  m_history.clear();

  m_transport = &driver;

  return true;
}

void StatelessWriter::reset() { m_is_initialized_ = false; }

const CacheChange *StatelessWriter::newChange(rtps::ChangeKind_t kind, const uint8_t *data,
                                              DataSize_t size, bool inLineQoS,
                                              bool markDisposedAfterWrite,
                                              bool hasRelatedSampleIdentity,
                                              const rpc::SampleIdentity &relatedSampleIdentity) {
  INIT_GUARD();
  if (isIrrelevant(kind)) {
    return nullptr;
  }
  std::lock_guard<std::recursive_mutex> lock(m_mutex);
  if (!m_is_initialized_) {
    return nullptr;
  }

  // A full history may drop its oldest sample on the next addChange: the static
  // ring always does, and the dynamic (host) ring does too when it hits its
  // 16-bit index ceiling and grow() refuses to resize. When a drop happens the
  // send cursor may point at a sequence number that no longer exists, which
  // would stall progress() permanently; advance it past the drop. When the
  // dynamic ring instead grows and RETAINS the oldest (the common host case) the
  // minimum is unchanged, so the cursor stays put and the retained sample is
  // still sent. Distinguish the two by whether the history minimum advanced
  // across the add - a copy, not a reference, because addChange may overwrite
  // the underlying slot.
  const bool wasFull = m_history.isFull();
  const SequenceNumber_t minBefore = m_history.getSeqNumMin();

  auto *result = m_history.addChange(data, size, inLineQoS, markDisposedAfterWrite,
                                     hasRelatedSampleIdentity, relatedSampleIdentity);

  if (wasFull) {
    const SequenceNumber_t minAfter = m_history.getSeqNumMin();
    if (minBefore < minAfter && m_nextSequenceNumberToSend < minAfter) {
      m_nextSequenceNumberToSend = minAfter; // Skip past the dropped sample
      SLW_LOG("History full, dropped oldest {}", this->m_attributes.topicName);
    }
  }
  if (m_transport != nullptr) {
    // Run the send asynchronously on the transport's worker pool (never inline
    // under the caller's locks), matching the previous ThreadPool semantics.
    // Guaranteed + banded: a bounded-queue rejection must not strand unsent
    // samples (a lone best-effort DATA has no recovery path), and a
    // prioritized endpoint's outbound work runs at ITS band end-to-end.
    m_transport->submitGuaranteed(
        this, [this]() { progress(); }, m_attributes.band);
  }

  SLW_LOG("Adding new data.");
  return result;
}

bool StatelessWriter::removeFromHistory(const SequenceNumber_t &s) {
  return false; // Stateless Writers currently do not support deletion from
                // history
}

void StatelessWriter::setAllChangesToUnsent() {
  INIT_GUARD();
  std::lock_guard<std::recursive_mutex> lock(m_mutex);

  m_nextSequenceNumberToSend = m_history.getSeqNumMin();

  if (m_transport != nullptr) {
    // Run the send asynchronously on the transport's worker pool (never inline
    // under the caller's locks), matching the previous ThreadPool semantics.
    // Guaranteed + banded: a bounded-queue rejection must not strand unsent
    // samples (a lone best-effort DATA has no recovery path), and a
    // prioritized endpoint's outbound work runs at ITS band end-to-end.
    m_transport->submitGuaranteed(
        this, [this]() { progress(); }, m_attributes.band);
  }
}

void StatelessWriter::onNewAckNack(const SubmessageAckNack & /*msg*/,
                                   const GuidPrefix_t &sourceGuidPrefix) {
  INIT_GUARD();
  // Too lazy to respond
}

void StatelessWriter::progress() {
  INIT_GUARD();
  // TODO smarter packaging e.g. by creating MessageStruct and serializing
  // after adjusting values.

  // Hold m_mutex across the proxy iteration: proxies are added/removed under
  // m_mutex from the SEDP receive workers, and iterating unlocked races those
  // mutations (see StatefulWriter::sendHeartBeat). m_mutex is recursive, so
  // the pre-existing inner history guard stays harmless.
  std::lock_guard<std::recursive_mutex> proxies_lock(m_mutex);
  if (m_proxies.getNumElements() == 0) {
    SLW_LOG("No proxy!");
  }

  for (const auto &proxy : m_proxies) {

    SLW_LOG("Progress.");
    // Do nothing, if someone else sends for me... (Multicast)
    if (proxy.useMulticast || !proxy.suppressUnicast || m_enforceUnicast) {
      PacketInfo info;
      info.srcPort = m_srcPort;
      PayloadBuffer payload;

      // Just usable for IPv4. Decide which locator to be used unicast/multicast.
      if (proxy.useMulticast && !m_enforceUnicast) {
        info.destAddr = proxy.remoteMulticastLocator.getIp4AddressBytes();
        info.destPort = (Ip4Port_t)proxy.remoteMulticastLocator.port;
      } else {
        info.destAddr = proxy.remoteLocator.getIp4AddressBytes();
        info.destPort = (Ip4Port_t)proxy.remoteLocator.port;
      }

      MessageFactory::addHeader(payload, m_attributes.endpointGuid.prefix);
      MessageFactory::addSubMessageTimeStamp(payload);

      {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        const CacheChange *next = m_history.getChangeBySN(m_nextSequenceNumberToSend);
        if (next == nullptr) {
          SLW_LOG("Couldn't get a new CacheChange with SN "
                  "(%li,%li)\n",
                  m_nextSequenceNumberToSend.high, m_nextSequenceNumberToSend.low);
          return;
        } else {
          SLW_LOG("Sending change with SN ({},{})", m_nextSequenceNumberToSend.high,
                  m_nextSequenceNumberToSend.low);
        }

        // Set EntityId to UNKNOWN if using multicast, because there might be
        // different ones...
        // TODO: mybe enhance by using UNKNOWN only if ids are really different
        EntityId_t reid;
        if (proxy.useMulticast && !m_enforceUnicast && proxy.unknown_eid) {
          reid = ENTITYID_UNKNOWN;
        } else {
          reid = proxy.remoteReaderGuid.entityId;
        }

        if (next->hasRelatedSampleIdentity) {
          // ROS 2 service request/reply: carry the related_sample_identity as
          // inline QoS. Such a sample must fit one DATA submessage - DATA_FRAG
          // carries no inline QoS, so fragmenting it would drop the correlation
          // and the caller would time out. The RPC facade rejects payloads above
          // MAX_UNFRAGMENTED_RPC_PAYLOAD; guard here too rather than emit an
          // uncorrelated fragment.
          if (next->data.spaceUsed() > MAX_UNFRAGMENTED_RPC_PAYLOAD) {
            continue;
          }
          MessageFactory::addSubMessageDataWithRelatedSampleIdentity(
              payload, next->data, next->relatedSampleIdentity, next->sequenceNumber,
              m_attributes.endpointGuid.entityId, reid);
        } else {
#ifdef RTPS_ENABLE_FRAGMENTATION
          if (next->data.spaceUsed() > MAX_UNFRAGMENTED_PAYLOAD) {
            // Oversized plain sample: emit DATA_FRAG submessages (built + sent
            // under the lock so next->data stays valid across fragments) and
            // skip the single DATA path for this proxy.
            sendSampleFragmented(info.destAddr, info.destPort, reid, next);
            continue;
          }
#endif
          MessageFactory::addSubMessageData(payload, next->data, false, next->sequenceNumber,
                                            m_attributes.endpointGuid.entityId,
                                            reid); // TODO
        }
      }

      info.payload = std::move(payload.bytes);
      SLW_LOG("Sending to {}.{}.{}.{}:{}", info.destAddr[0], info.destAddr[1], info.destAddr[2],
              info.destAddr[3], info.destPort);
      if (info.payload.empty()) {
        continue;
      }
      m_transport->sendPacket(info);
    }
  }

  m_history.removeUntilIncl(m_nextSequenceNumberToSend);
  ++m_nextSequenceNumberToSend;
}

#ifdef RTPS_ENABLE_FRAGMENTATION
bool StatelessWriter::sendSampleFragmented(const Ip4AddressBytes &destAddr, Ip4Port_t destPort,
                                           const EntityId_t &readerId, const CacheChange *next) {
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
