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

#ifndef RTPS_THREADPOOL_H
#define RTPS_THREADPOOL_H

#include "base_component.hpp"
#include "rtps/communication/PacketInfo.h"
#include "rtps/common/types.h"
#include "rtps/config.h"
#include "rtps/storages/ThreadSafeCircularBuffer.h"

#include <array>
#include <cstddef>
#include <memory>

namespace rtps {

class Writer;

} // namespace rtps

namespace espp {
class ThreadPool;
}

namespace rtps {

class ThreadPool : public espp::BaseComponent {
public:
  using receiveJumppad_fp = void (*)(void *callee, const PacketInfo &packet);

  ThreadPool(receiveJumppad_fp receiveCallback, void *callee);

  ~ThreadPool();

  bool startThreads();
  void stopThreads();

  void clearQueues();
  bool addWorkload(Writer *workload);
  bool addNewPacket(PacketInfo &&packet);

  static void onDatagram(
      void *arg, const uint8_t *data, std::size_t size, Ip4Port_t localPort,
      Ip4Port_t remotePort,
      const Ip4AddressBytes &remoteAddress);

  bool addBuiltinPort(const Ip4Port_t &port);

private:
  receiveJumppad_fp m_receiveJumppad;
  void *m_callee;
  bool m_running = false;
    std::unique_ptr<espp::ThreadPool> m_writerWorkers;
    std::unique_ptr<espp::ThreadPool> m_readerWorkers;

  std::array<Ip4Port_t, 2 * Config::MAX_NUM_PARTICIPANTS> m_builtinPorts;
  size_t m_builtinPortsIdx = 0;

  void updateDiagnostics();

  using BufferUsertrafficOutgoing = ThreadSafeCircularBuffer<
      Writer *, Config::THREAD_POOL_WORKLOAD_QUEUE_LENGTH_USERTRAFFIC>;
  using BufferMetatrafficOutgoing = ThreadSafeCircularBuffer<
      Writer *, Config::THREAD_POOL_WORKLOAD_QUEUE_LENGTH_METATRAFFIC>;
  using BufferUsertrafficIncoming = ThreadSafeCircularBuffer<
      PacketInfo, Config::THREAD_POOL_WORKLOAD_QUEUE_LENGTH_USERTRAFFIC>;
  using BufferMetatrafficIncoming = ThreadSafeCircularBuffer<
      PacketInfo, Config::THREAD_POOL_WORKLOAD_QUEUE_LENGTH_METATRAFFIC>;

  BufferUsertrafficOutgoing m_outgoingUserTraffic;
  BufferMetatrafficOutgoing m_outgoingMetaTraffic;

  BufferUsertrafficIncoming m_incomingUserTraffic;
  BufferMetatrafficIncoming m_incomingMetaTraffic;

    void scheduleWriterWork();
    void scheduleReaderWork();

  bool isBuiltinPort(const Ip4Port_t &port);
  void doWriterWork();
  void doReaderWork();
};
} // namespace rtps

#endif // RTPS_THREADPOOL_H
