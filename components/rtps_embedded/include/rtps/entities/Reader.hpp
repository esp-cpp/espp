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

#ifndef RTPS_READER_H
#define RTPS_READER_H

#include "base_component.hpp"
#include "rtps/common/types.hpp"
#include "rtps/config.hpp"
#include "rtps/discovery/TopicData.hpp"
#include "rtps/entities/WriterProxy.hpp"
#include "rtps/rpc/sample_identity.hpp"
#include "rtps/storages/MemoryPool.hpp"
#include <cstring>
#include <mutex>
#ifdef RTPS_ENABLE_FRAGMENTATION
#include <vector>
#endif

namespace rtps {

struct SubmessageHeartbeat;
struct SubmessageGap;

class ReaderCacheChange {
private:
  const uint8_t *data;

public:
  const ChangeKind_t kind;
  const DataSize_t size;
  const Guid_t writerGuid;
  const SequenceNumber_t sn;
  // ROS 2 request/reply correlation: when the DATA carried a
  // related_sample_identity inline QoS (PID 0x0083 / 0x800f), it is surfaced here
  // so a service endpoint can correlate. Zero-cost for plain pub/sub (the flag
  // stays false). See rpc/sample_identity.hpp.
  const bool hasRelatedSampleIdentity;
  const rpc::SampleIdentity relatedSampleIdentity;

  ReaderCacheChange(ChangeKind_t kind, Guid_t &writerGuid, SequenceNumber_t sn, const uint8_t *data,
                    DataSize_t size, bool hasRelatedSampleIdentity = false,
                    const rpc::SampleIdentity &relatedSampleIdentity = {})
      : data(data)
      , kind(kind)
      , size(size)
      , writerGuid(writerGuid)
      , sn(sn)
      , hasRelatedSampleIdentity(hasRelatedSampleIdentity)
      , relatedSampleIdentity(relatedSampleIdentity){};

  ~ReaderCacheChange() = default; // No need to free data. It's not owned by this object
  // Not allowed because this class doesn't own the ptr and the user isn't
  // allowed to use it outside the Scope of the callback
  ReaderCacheChange(const ReaderCacheChange &other) = delete;
  ReaderCacheChange(ReaderCacheChange &&other) = delete;
  ReaderCacheChange &operator=(const ReaderCacheChange &other) = delete;
  ReaderCacheChange &operator=(ReaderCacheChange &&other) = delete;

  bool copyInto(uint8_t *buffer, DataSize_t destSize) const {
    if (destSize < size) {
      return false;
    } else {
      memcpy(buffer, data, size);
      return true;
    }
  }

  const uint8_t *getData() const { return data; }

  DataSize_t getDataSize() const { return size; }
};

typedef void (*ddsReaderCallback_fp)(void *callee, const ReaderCacheChange &cacheChange);

class Reader : public espp::BaseComponent {
public:
  using callbackFunction_t = void (*)(void *, const ReaderCacheChange &);
  using callbackIdentifier_t = uint32_t;

  TopicData m_attributes;
  virtual void newChange(const ReaderCacheChange &cacheChange) = 0;
  virtual callbackIdentifier_t registerCallback(callbackFunction_t cb, void *arg);
  virtual bool removeCallback(callbackIdentifier_t identifier);
  uint8_t getNumCallbacks();

  virtual bool onNewHeartbeat(const SubmessageHeartbeat &msg, const GuidPrefix_t &remotePrefix) = 0;
  virtual bool onNewGapMessage(const SubmessageGap &msg, const GuidPrefix_t &remotePrefix) = 0;
  virtual bool addNewMatchedWriter(const WriterProxy &newProxy) = 0;
  virtual bool removeProxy(const Guid_t &guid);
  virtual void removeAllProxiesOfParticipant(const GuidPrefix_t &guidPrefix);
  bool isInitialized() const { return m_is_initialized_; }
  virtual void reset();
  bool isProxy(const Guid_t &guid);
  WriterProxy *getProxy(Guid_t guid);
  uint32_t getProxiesCount();

  void setSEDPSequenceNumber(const SequenceNumber_t &sn);
  const SequenceNumber_t &getSEDPSequenceNumber();

  using dumpProxyCallback = void (*)(const Reader *reader, const WriterProxy &, void *arg);

  int dumpAllProxies(dumpProxyCallback target, void *arg);

  virtual bool sendPreemptiveAckNack(const WriterProxy &writer);

#ifdef RTPS_ENABLE_FRAGMENTATION
  /// Accumulate one DATA_FRAG fragment (best-effort reassembly). When all
  /// fragments of the sample identified by (writerGuid, sn) have arrived, the
  /// completed sample is delivered through the normal newChange() path (same as
  /// a single DATA). A newer sample evicts an older incomplete one; samples
  /// larger than Config::MAX_SAMPLE_SIZE are refused. Thread-safe.
  void newFragment(const Guid_t &writerGuid, const SequenceNumber_t &sn,
                   uint32_t fragmentStartingNum, uint16_t fragmentsInSubmessage,
                   uint16_t fragmentSize, uint32_t sampleSize, const uint8_t *fragData,
                   DataSize_t fragDataLen);
#endif

protected:
  void executeCallbacks(const ReaderCacheChange &cacheChange);
  bool initMutex();

  SequenceNumber_t m_sedp_sequence_number;

  bool m_is_initialized_ = false;
  Reader();
  virtual ~Reader() = default;
  MemoryPool<WriterProxy, Config::NUM_WRITER_PROXIES_PER_READER> m_proxies;

  callbackIdentifier_t m_callback_identifier = 1;

  uint8_t m_callback_count = 0;
  using callbackElement_t = struct {
    callbackFunction_t function;
    void *arg;
    callbackIdentifier_t identifier;
  };

  std::array<callbackElement_t, Config::MAX_NUM_READER_CALLBACKS> m_callbacks;

  // Guards manipulation of the proxies array
  std::recursive_mutex m_proxies_mutex;

  // Guards manipulation of callback array
  std::recursive_mutex m_callback_mutex;

#ifdef RTPS_ENABLE_FRAGMENTATION
  // Single best-effort reassembly slot: accumulates the fragments of one sample
  // at a time (a newer sample or different writer evicts an older incomplete
  // one). Bounded by Config::MAX_SAMPLE_SIZE.
  struct Reassembly {
    bool active = false;
    Guid_t writerGuid{};
    SequenceNumber_t sn{};
    uint32_t sampleSize = 0;
    uint16_t fragmentSize = 0;
    uint32_t totalFragments = 0;
    uint32_t receivedFragments = 0;
    std::vector<uint8_t> buffer;
    std::vector<bool> received;
  };
  Reassembly m_reassembly;
  std::mutex m_reassembly_mutex;
#endif
};
} // namespace rtps

#endif // RTPS_READER_H
