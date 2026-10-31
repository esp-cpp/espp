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

#ifndef RTPS_STATEFULREADER_H
#define RTPS_STATEFULREADER_H

#include "rtps/common/types.hpp"
#include "rtps/communication/PacketInfo.hpp"
#include "rtps/config.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/WriterProxy.hpp"
#include "rtps/storages/MemoryPool.hpp"
#include <mutex>

namespace rtps {
class EsppTransport;
struct SubmessageHeartbeat;

class StatefulReader final : public Reader {
public:
  StatefulReader()
      : m_srcPort(0)
      , m_transport(nullptr) {}
  ~StatefulReader() override;
  bool init(const TopicData &attributes, EsppTransport &driver);
  void newChange(const ReaderCacheChange &cacheChange) override;
  bool addNewMatchedWriter(const WriterProxy &newProxy) override;
  bool onNewHeartbeat(const SubmessageHeartbeat &msg, const GuidPrefix_t &remotePrefix) override;
  bool onNewGapMessage(const SubmessageGap &msg, const GuidPrefix_t &remotePrefix) override;

  bool sendPreemptiveAckNack(const WriterProxy &writer) override;

private:
  Ip4Port_t m_srcPort; // TODO intended for reuse but buffer not used as such
  EsppTransport *m_transport;
  /// Serializes sample DELIVERY (the expectedSN claim + the user callbacks)
  /// without holding m_proxies_mutex across user code. A leaf in the lock
  /// order: newChange() acquires it FIRST and only nests m_proxies_mutex
  /// briefly inside for the claim; nothing acquires it while holding any other
  /// engine/facade lock. This preserves the strict in-order,
  /// one-callback-at-a-time semantics the proxies mutex used to provide while
  /// keeping user callbacks (which may call back into the facade and from
  /// there into SEDP/participant/proxies locks) off the proxies mutex -
  /// breaking the callback->facade->SEDP->proxies lock-order cycle.
  std::mutex m_delivery_mutex;
};

} // namespace rtps

#endif // RTPS_STATEFULREADER_H
