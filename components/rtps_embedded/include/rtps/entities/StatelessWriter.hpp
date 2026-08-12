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

#ifndef RTPS_RTPSWRITER_H
#define RTPS_RTPSWRITER_H

#include "rtps/common/types.hpp"
#include "rtps/config.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/storages/MemoryPool.hpp"
#include "rtps/storages/SimpleHistoryCache.hpp"

namespace rtps {

class EsppTransport;

class StatelessWriter : public Writer {
public:
  StatelessWriter()
      : m_transport(nullptr) {}
  ~StatelessWriter() override;
  bool init(TopicData attributes, TopicKind_t topicKind, EsppTransport &driver,
            bool enfUnicast = false);

  void progress() override;
  const CacheChange *newChange(ChangeKind_t kind, const uint8_t *data, DataSize_t size,
                               bool inLineQoS = false,
                               bool markDisposedAfterWrite = false) override;
  bool removeFromHistory(const SequenceNumber_t &s);

  void setAllChangesToUnsent() override;
  void onNewAckNack(const SubmessageAckNack &msg, const GuidPrefix_t &sourceGuidPrefix) override;
  void reset() override;

private:
  EsppTransport *m_transport;

  SimpleHistoryCache<Config::HISTORY_SIZE_STATELESS> m_history;
};

} // namespace rtps

#endif // RTPS_RTPSWRITER_H
