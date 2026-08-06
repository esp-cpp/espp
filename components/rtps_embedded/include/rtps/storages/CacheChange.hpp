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

#ifndef PROJECT_CACHECHANGE_H
#define PROJECT_CACHECHANGE_H

#include "rtps/common/types.hpp"
#include "rtps/storages/PayloadBuffer.hpp"

#include <chrono>

namespace rtps {
struct CacheChange {
  using TimePoint = std::chrono::steady_clock::time_point;

  ChangeKind_t kind = ChangeKind_t::INVALID;
  bool inLineQoS = false;
  bool disposeAfterWrite = false;
  TimePoint sentTime{};
  SequenceNumber_t sequenceNumber = SEQUENCENUMBER_UNKNOWN;
  PayloadBuffer data;

  CacheChange &operator=(const CacheChange &other) = delete;

  CacheChange &operator=(CacheChange &&other) noexcept {
    kind = other.kind;
    inLineQoS = other.inLineQoS;
    disposeAfterWrite = other.disposeAfterWrite;
    sentTime = other.sentTime;
    sequenceNumber = other.sequenceNumber;
    data = std::move(other.data);
    return *this;
  }

  CacheChange() = default;
  CacheChange(ChangeKind_t kind, SequenceNumber_t sequenceNumber)
      : kind(kind)
      , sequenceNumber(sequenceNumber){};

  void reset() {
    kind = ChangeKind_t::INVALID;
    sequenceNumber = SEQUENCENUMBER_UNKNOWN;
    inLineQoS = false;
    disposeAfterWrite = false;
    sentTime = TimePoint{};
  }

  bool isInitialized() { return (kind != ChangeKind_t::INVALID); }
};
} // namespace rtps

#endif // PROJECT_CACHECHANGE_H
