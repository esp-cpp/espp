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

#include "rtps/entities/StatelessReader.hpp"

#include "rtps/utils/Log.hpp"
#include <algorithm>
#include <limits>

using rtps::StatelessReader;

#if SLR_VERBOSE && RTPS_GLOBAL_VERBOSE
#include "rtps/utils/printutils.hpp"
#define SLR_LOG(...)                                                                               \
  if (true) {                                                                                      \
    printf("[StatelessReader %s] ", &m_attributes.topicName[0]);                                   \
    printf(__VA_ARGS__);                                                                           \
    printf("\n");                                                                                  \
  }
#else
#define SLR_LOG(...) //
#endif

bool StatelessReader::init(const TopicData &attributes) {
  if (!initMutex()) {
    return false;
  }

  m_proxies.clear();
  m_attributes = attributes;
  m_is_initialized_ = true;
  return true;
}

void StatelessReader::newChange(const ReaderCacheChange &cacheChange) {
  if (!m_is_initialized_) {
    return;
  }
  // Best effort still rejects what arrives out of order: RTPS 2.5 8.4.12.1
  // has the best-effort reader ignore a change whose sequence number is not
  // above the highest already received from that writer, so a duplicated or
  // reordered datagram never reaches the application twice, or late. Here
  // expectedSN is "the next sequence number from this writer"; it jumps
  // forward over a gap (nothing is ever requested again) and a change below
  // it is dropped. A change from a writer without a proxy is delivered as
  // before. Found by the QEMU interop matrix's duplication and reordering
  // cases (components/rtps/interop/qemu).
  //
  // The claim and the callbacks are one operation under m_delivery_mutex (as
  // in StatefulReader::newChange()): receive workers for different sockets
  // can hand this reader consecutive samples at once, and with the callbacks
  // unlocked one could claim SN 1, pause, and deliver after the other had
  // delivered SN 2. The delivery mutex is a leaf taken first; the proxies
  // mutex is still released before any user code runs.
  std::lock_guard<std::mutex> delivery(m_delivery_mutex);
  {
    std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
    auto matches_writer = [&](const WriterProxy &proxy) {
      return proxy.remoteWriterGuid == cacheChange.writerGuid;
    };
    auto it = std::find_if(m_proxies.begin(), m_proxies.end(), matches_writer);
    if (it != m_proxies.end()) {
      if (cacheChange.sn < it->expectedSN) {
        return; // a duplicate, or older than something already delivered
      }
      it->expectedSN = cacheChange.sn;
      // sn + 1, unless sn is the last representable value: the number came
      // off the wire, and incrementing past the top would overflow the
      // signed high word. At the top the next expected stays at sn, so a
      // repeat of that one last number would be delivered again; nothing
      // real gets there (2^63 samples).
      const bool at_top = cacheChange.sn.high == std::numeric_limits<int32_t>::max() &&
                          cacheChange.sn.low == std::numeric_limits<uint32_t>::max();
      if (!at_top) {
        ++it->expectedSN;
      }
    }
  }
  executeCallbacks(cacheChange);
}

bool StatelessReader::addNewMatchedWriter(const WriterProxy &newProxy) {
#if (SLR_VERBOSE && RTPS_GLOBAL_VERBOSE)
  SLR_LOG("Adding WriterProxy");
  printGuid(newProxy.remoteWriterGuid);
#endif
  // Guard the pool mutation against concurrent m_proxies iteration (see
  // StatefulReader::addNewMatchedWriter).
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  return m_proxies.add(newProxy);
}

bool StatelessReader::onNewHeartbeat(const SubmessageHeartbeat &, const GuidPrefix_t &) {
  // nothing to do
  return true;
}

bool StatelessReader::onNewGapMessage(const SubmessageGap &msg, const GuidPrefix_t &remotePrefix) {
  return true;
}

#undef SLR_VERBOSE
