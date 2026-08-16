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

#ifndef RTPS_WRITER_H
#define RTPS_WRITER_H

#include "base_component.hpp"
#include "rtps/discovery/TopicData.hpp"
#include "rtps/entities/ReaderProxy.hpp"
#include "rtps/storages/CacheChange.hpp"
#include "rtps/storages/MemoryPool.hpp"

#include <cstdint>
#include <mutex>

#ifdef DEBUG_BUILD
#define COMPILE_INIT_GUARD
#endif

#ifdef COMPILE_INIT_GUARD
#define INIT_GUARD()                                                                               \
  if (!m_is_initialized_) {                                                                        \
    logger_.error("Using uninitialized endpoint");                                                 \
    while (1) {                                                                                    \
    }                                                                                              \
  }                                                                                                \
  }
#else
#define INIT_GUARD() //
#endif

namespace rtps {

class Writer : public espp::BaseComponent {
public:
  TopicData m_attributes;
  virtual bool addNewMatchedReader(const ReaderProxy &newProxy);
  virtual bool removeProxy(const Guid_t &guid);
  virtual void removeAllProxiesOfParticipant(const GuidPrefix_t &guidPrefix);
  virtual void reset() = 0;
  virtual const CacheChange *newChange(ChangeKind_t kind, const uint8_t *data, DataSize_t size);

  //! Add a change that will be sent as a DATA carrying relatedSampleIdentity as
  //! inline QoS (ROS 2 service request/reply correlation). Used by the service
  //! request/reply writers; plain pub/sub uses the forms above and is unaffected.
  const CacheChange *
  newChangeWithRelatedSampleIdentity(ChangeKind_t kind, const uint8_t *data, DataSize_t size,
                                     const rpc::SampleIdentity &relatedSampleIdentity) {
    return newChange(kind, data, size, /*inLineQoS=*/false, /*markDisposedAfterWrite=*/false,
                     /*hasRelatedSampleIdentity=*/true, relatedSampleIdentity);
  }

  //! Executes required steps like sending packets. Intended to be called by
  //! worker threads
  virtual void progress() = 0;

  virtual bool removeFromHistory(const SequenceNumber_t &s) = 0;
  virtual void setAllChangesToUnsent() = 0;
  virtual void onNewAckNack(const SubmessageAckNack &msg, const GuidPrefix_t &sourceGuidPrefix) = 0;

  using dumpProxyCallback = void (*)(const Writer *writer, const ReaderProxy &, void *arg);

  int dumpAllProxies(dumpProxyCallback target, void *arg);

  bool isInitialized();
  std::uint32_t getProxiesCount();

  void setSEDPSequenceNumber(const SequenceNumber_t &sn);
  const SequenceNumber_t &getSEDPSequenceNumber();

  bool isBuiltinEndpoint();

  /// Set the nominal per-fragment payload size used when a published sample is
  /// too large for a single DATA submessage and must be split into DATA_FRAG
  /// submessages. Clamped to <= MAX_FRAGMENT_SIZE so each single-fragment
  /// DATA_FRAG still fits one UDP datagram. No effect when fragmentation is
  /// compiled out.
  void setFragmentSize(uint16_t fragmentSize) { m_fragmentSize = fragmentSize; }

protected:
  Writer();
  SequenceNumber_t m_sedp_sequence_number;

  //! Nominal per-fragment payload size for DATA_FRAG (default 63000).
  uint16_t m_fragmentSize = 63000;

  std::recursive_mutex m_mutex;

  Ip4Port_t m_srcPort;

  bool m_enforceUnicast;

  TopicKind_t m_topicKind = TopicKind_t::NO_KEY;
  SequenceNumber_t m_nextSequenceNumberToSend;

  friend class SEDPAgent;
  virtual const CacheChange *newChange(ChangeKind_t kind, const uint8_t *data, DataSize_t size,
                                       bool inLineQoS, bool markDisposedAfterWrite,
                                       bool hasRelatedSampleIdentity = false,
                                       const rpc::SampleIdentity &relatedSampleIdentity = {}) = 0;

  friend class SizeInspector;
  bool m_is_initialized_ = false;
  virtual ~Writer() = default;
  MemoryPool<ReaderProxy, Config::NUM_READER_PROXIES_PER_WRITER> m_proxies;

  void resetSendOptions();
  void manageSendOptions();
  bool isIrrelevant(ChangeKind_t kind) const;
};
} // namespace rtps

#endif // RTPS_WRITER_H
