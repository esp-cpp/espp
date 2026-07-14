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

#include "rtps/ThreadPool.h"

#include "rtps/entities/Domain.h"
#include "rtps/entities/Writer.h"
#include "rtps/utils/Diagnostics.h"
#include "rtps/utils/Log.h"
#include "rtps/utils/udpUtils.h"
#include "thread_pool.hpp"

using rtps::ThreadPool;

#if THREAD_POOL_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.h"
#define THREAD_POOL_LOG(...)                                                   \
  if (true) {                                                                  \
    printf("[ThreadPool] ");                                                   \
    printf(__VA_ARGS__);                                                       \
    printf("\r\n");                                                            \
  }
#else
#define THREAD_POOL_LOG(...) //
#endif

ThreadPool::ThreadPool(receiveJumppad_fp receiveCallback, void *callee)
    : m_receiveJumppad(receiveCallback), m_callee(callee) {

  if (!m_outgoingMetaTraffic.init() || !m_outgoingUserTraffic.init() ||
      !m_incomingMetaTraffic.init() || !m_incomingUserTraffic.init()) {
    return;
  }

  m_writerWorkers = std::make_unique<espp::ThreadPool>(espp::ThreadPool::Config{
      .worker_count = Config::THREAD_POOL_NUM_WRITERS,
      .max_queue_size = 0,
      .auto_start = false,
      .block_on_submit_when_full = false,
      .worker_task_config = {
          .name = "rtps_writer",
          .stack_size_bytes = static_cast<size_t>(Config::THREAD_POOL_WRITER_STACKSIZE),
          .priority = static_cast<size_t>(Config::THREAD_POOL_WRITER_PRIO),
          .core_id = -1,
      },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  m_readerWorkers = std::make_unique<espp::ThreadPool>(espp::ThreadPool::Config{
      .worker_count = Config::THREAD_POOL_NUM_READERS,
      .max_queue_size = 0,
      .auto_start = false,
      .block_on_submit_when_full = false,
      .worker_task_config = {
          .name = "rtps_reader",
          .stack_size_bytes = static_cast<size_t>(Config::THREAD_POOL_READER_STACKSIZE),
          .priority = static_cast<size_t>(Config::THREAD_POOL_READER_PRIO),
          .core_id = -1,
      },
      .log_level = espp::Logger::Verbosity::WARN,
  });
}

ThreadPool::~ThreadPool() {
  if (m_running) {
    stopThreads();
  }
}

void ThreadPool::updateDiagnostics() {

  rtps::Diagnostics::ThreadPool::max_ever_elements_incoming_usertraffic_queue =
      std::max(rtps::Diagnostics::ThreadPool::
                   max_ever_elements_incoming_usertraffic_queue,
               m_incomingUserTraffic.numElements());

  rtps::Diagnostics::ThreadPool::max_ever_elements_outgoing_usertraffic_queue =
      std::max(rtps::Diagnostics::ThreadPool::
                   max_ever_elements_outgoing_usertraffic_queue,
               m_outgoingUserTraffic.numElements());

  rtps::Diagnostics::ThreadPool::max_ever_elements_incoming_metatraffic_queue =
      std::max(rtps::Diagnostics::ThreadPool::
                   max_ever_elements_incoming_metatraffic_queue,
               m_incomingMetaTraffic.numElements());

  rtps::Diagnostics::ThreadPool::max_ever_elements_outgoing_metatraffic_queue =
      std::max(rtps::Diagnostics::ThreadPool::
                   max_ever_elements_outgoing_metatraffic_queue,
               m_outgoingMetaTraffic.numElements());
}

bool ThreadPool::startThreads() {
  if (m_running) {
    return true;
  }
  if (!m_writerWorkers || !m_readerWorkers) {
    return false;
  }

  m_running = true;
  m_writerWorkers->start();
  m_readerWorkers->start();

  scheduleWriterWork();
  scheduleReaderWork();
  return true;
}

void ThreadPool::stopThreads() {
  m_running = false;
  if (m_writerWorkers) {
    m_writerWorkers->stop();
  }
  if (m_readerWorkers) {
    m_readerWorkers->stop();
  }
}

void ThreadPool::clearQueues() {
  m_outgoingMetaTraffic.clear();
  m_outgoingUserTraffic.clear();
  m_incomingMetaTraffic.clear();
  m_incomingUserTraffic.clear();
}

bool ThreadPool::addWorkload(Writer *workload) {
  bool res = false;
  if (workload->isBuiltinEndpoint()) {
    res = m_outgoingMetaTraffic.moveElementIntoBuffer(std::move(workload));
  } else {
    res = m_outgoingUserTraffic.moveElementIntoBuffer(std::move(workload));
  }
  if (!res) {
	if(workload->isBuiltinEndpoint()){
		rtps::Diagnostics::ThreadPool::dropped_outgoing_packets_metatraffic++;
	}else{
		rtps::Diagnostics::ThreadPool::dropped_outgoing_packets_usertraffic++;
	}
    THREAD_POOL_LOG("Failed to enqueue outgoing packet.");
    return false;
  }

  scheduleWriterWork();

  return res;
}

bool ThreadPool::addBuiltinPort(const Ip4Port_t &port) {
  if (m_builtinPortsIdx == m_builtinPorts.size()) {
    return false;
  }

  // TODO: Does not allow for participant deletion!
  m_builtinPorts[m_builtinPortsIdx] = port;
  m_builtinPortsIdx++;

  return true;
}

bool ThreadPool::isBuiltinPort(const Ip4Port_t &port) {
  if (getBuiltInMulticastLocator().port == port) {
    return true;
  }

  for (unsigned int i = 0; i < m_builtinPortsIdx; i++) {
    if (m_builtinPorts[i] == port) {
      return true;
    }
  }

  return false;
}

bool ThreadPool::addNewPacket(PacketInfo &&packet) {
  bool res = false;
  if (isBuiltinPort(packet.destPort)) {
    res = m_incomingMetaTraffic.moveElementIntoBuffer(std::move(packet));
  } else {
    res = m_incomingUserTraffic.moveElementIntoBuffer(std::move(packet));
  }
  if (!res) {
    THREAD_POOL_LOG("failed to enqueue packet for port %u",
                    static_cast<unsigned int>(packet.destPort));
    return false;
  }

  scheduleReaderWork();
  return res;
}

void ThreadPool::scheduleWriterWork() {
  if (!m_running || !m_writerWorkers) {
    return;
  }

  (void)m_writerWorkers->try_submit([this]() { doWriterWork(); });
}

void ThreadPool::doWriterWork() {
  while (m_running) {
    Writer *workload_usertraffic = nullptr;
    bool workload_usertraffic_available = m_outgoingUserTraffic.moveFirstInto(workload_usertraffic);
    if (workload_usertraffic_available) {
      workload_usertraffic->progress();
      Diagnostics::ThreadPool::processed_outgoing_usertraffic++;
    }

    Writer *workload_metatraffic = nullptr;
    bool workload_metatraffic_available = m_outgoingMetaTraffic.moveFirstInto(workload_metatraffic);
    if (workload_metatraffic_available) {
      workload_metatraffic->progress();
      Diagnostics::ThreadPool::processed_outgoing_metatraffic++;
    }

    if (workload_usertraffic_available || workload_metatraffic_available) {
      continue;
    }

    // THREAD_POOL_LOG("WriterWorker | User = %u, Meta = %u\r\n",
    //                 static_cast<unsigned int>(Diagnostics::ThreadPool::processed_outgoing_usertraffic),
    //                 static_cast<unsigned int>(Diagnostics::ThreadPool::processed_outgoing_metatraffic));
    updateDiagnostics();
    return;
  }
}

void ThreadPool::onDatagram(
    void *arg, const uint8_t *data, std::size_t size, Ip4Port_t localPort,
    Ip4Port_t remotePort,
    const platform::transport::Ip4AddressBytes &remoteAddress) {
  auto &pool = *static_cast<ThreadPool *>(arg);

  PacketInfo packet;
  packet.destAddr = remoteAddress;
  packet.destPort = localPort;
  packet.srcPort = remotePort;

  if (size > 0 && data != nullptr) {
    packet.payload.assign(data, data + size);
  }

  if (!pool.addNewPacket(std::move(packet))) {
    THREAD_POOL_LOG("ThreadPool: dropped packet\n");
    if (pool.isBuiltinPort(remotePort)) {
      rtps::Diagnostics::ThreadPool::dropped_incoming_packets_metatraffic++;
    } else {
      rtps::Diagnostics::ThreadPool::dropped_incoming_packets_usertraffic++;
    }
  }
}

void ThreadPool::scheduleReaderWork() {
  if (!m_running || !m_readerWorkers) {
    return;
  }

  (void)m_readerWorkers->try_submit([this]() { doReaderWork(); });
}

void ThreadPool::doReaderWork() {
  uint32_t metatraffic = 0;
  uint32_t usertraffic = 0;
  while (m_running) {
    PacketInfo packet_user;
    auto isUserWorkToDo = m_incomingUserTraffic.moveFirstInto(packet_user);
    if (isUserWorkToDo) {
      Diagnostics::ThreadPool::processed_incoming_usertraffic++;
      m_receiveJumppad(m_callee, const_cast<const PacketInfo &>(packet_user));
    }

    PacketInfo packet_meta;
    auto isMetaWorkToDo = m_incomingMetaTraffic.moveFirstInto(packet_meta);
    if (isMetaWorkToDo) {
      Diagnostics::ThreadPool::processed_incoming_metatraffic++;
      m_receiveJumppad(m_callee, const_cast<const PacketInfo &>(packet_meta));
    }

    if (isUserWorkToDo || isMetaWorkToDo) {
      continue;
    }
    THREAD_POOL_LOG("ReaderWorker | User = %u, Meta = %u\r\n",
                    static_cast<unsigned int>(Diagnostics::ThreadPool::processed_incoming_usertraffic),
                    static_cast<unsigned int>(Diagnostics::ThreadPool::processed_incoming_metatraffic));
    updateDiagnostics();
    return;
  }
}

#undef THREAD_POOL_VERBOSE
