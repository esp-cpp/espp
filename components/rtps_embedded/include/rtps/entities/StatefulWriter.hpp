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

#ifndef RTPS_STATEFULWRITER_H
#define RTPS_STATEFULWRITER_H

#include "rtps/common/types.hpp"
#include "rtps/communication/PacketInfo.hpp"
#include "rtps/entities/ReaderProxy.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/storages/HistoryCacheWithDeletion.hpp"
#include "rtps/storages/MemoryPool.hpp"
#include "rtps/storages/ThreadSafeCircularBuffer.hpp"
#include "task.hpp"

#include <memory>

namespace rtps {

class EsppTransport;

template <class NetworkDriver> class StatefulWriterT final : public Writer {
public:
  StatefulWriterT()
      : m_transport(nullptr) {}
  ~StatefulWriterT() override;
  bool init(TopicData attributes, TopicKind_t topicKind, NetworkDriver &driver,
            bool enfUnicast = false);

  //! Executes required steps like sending packets. Intended to be called by
  //! worker threads
  void progress() override;
  const CacheChange *newChange(ChangeKind_t kind, const uint8_t *data, DataSize_t size,
                               bool inLineQoS = false,
                               bool markDisposedAfterWrite = false) override;

  bool removeFromHistory(const SequenceNumber_t &s);

  /// Run one heartbeat evaluation, preserving the historical cadence: sends a
  /// HEARTBEAT when due (period/4 while any proxy has unacknowledged changes,
  /// full period otherwise), drops delayed dispose-after-write changes, and
  /// returns the next time this writer wants to be ticked. Called by the
  /// Domain's protocol scheduler task instead of a dedicated per-writer
  /// heartbeat thread.
  std::chrono::steady_clock::time_point heartbeatTick(std::chrono::steady_clock::time_point now);

  /// Install the scheduler nudge: invoked on newChange() so a publish
  /// piggybacks an immediate heartbeat evaluation instead of waiting out the
  /// current period.
  void setProtocolNudge(std::function<void()> nudge) { m_protocolNudge = std::move(nudge); }
  void setAllChangesToUnsent() override;
  void onNewAckNack(const SubmessageAckNack &msg, const GuidPrefix_t &sourceGuidPrefix) override;
  void reset() override;
  void updateChangeKind(SequenceNumber_t &sequence_number);

private:
  NetworkDriver *m_transport = nullptr;

  HistoryCacheWithDeletion<Config::HISTORY_SIZE_STATEFUL> m_history;

  /*
   * Cache changes marked as disposeAfterWrite are retained for a short amount
   * in case of retransmission The whole 'disposeAfterWrite' mechanisms only
   * exists to allow for repeated creation and deletion of endpoints during
   * operation. Otherwise the history will quickly reach its limits. Will be
   * replaced with something more elegant in the future.
   */
  ThreadSafeCircularBuffer<SequenceNumber_t, 10> m_disposeWithDelay;
  void dropDisposeAfterWriteChanges();

  Count_t m_hbCount{1};

  /// Next heartbeat deadline (managed by heartbeatTick / newChange).
  std::chrono::steady_clock::time_point m_nextHeartbeat{};
  std::function<void()> m_protocolNudge{};

  bool sendData(const ReaderProxy &reader, const CacheChange *next);
  bool sendDataWRMulticast(const ReaderProxy &reader, const CacheChange *next);
  void sendHeartBeat();
  void sendGap(const ReaderProxy &reader, const SequenceNumber_t &firstMissing,
               const SequenceNumber_t &nextValid);
};

using StatefulWriter = StatefulWriterT<EsppTransport>;
} // namespace rtps

#include "StatefulWriter.tpp"

#endif // RTPS_STATEFULWRITER_H
