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

#ifndef RTPS_STATELESSREADER_H
#define RTPS_STATELESSREADER_H

#include "rtps/entities/Reader.hpp"

#include <mutex>

namespace rtps {
class StatelessReader final : public Reader {
public:
  bool init(const TopicData &attributes);
  void newChange(const ReaderCacheChange &cacheChange) override;
  bool onNewHeartbeat(const SubmessageHeartbeat &msg, const GuidPrefix_t &remotePrefix) override;
  bool addNewMatchedWriter(const WriterProxy &newProxy) override;
  bool onNewGapMessage(const SubmessageGap &msg, const GuidPrefix_t &remotePrefix) override;

private:
  /// Serializes newChange()'s claim of a sequence number and the callbacks it
  /// delivers, so samples reach the application in claim order even when
  /// receive workers for different sockets hand them in concurrently (see
  /// StatefulReader::m_delivery_mutex).
  std::mutex m_delivery_mutex;
};

} // namespace rtps

#endif // RTPS_STATELESSREADER_H
