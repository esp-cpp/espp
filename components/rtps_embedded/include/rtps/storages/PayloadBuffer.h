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

#ifndef RTPS_PAYLOADBUFFER_H
#define RTPS_PAYLOADBUFFER_H

#include <vector>

#include "rtps/common/types.h"

namespace rtps {

struct PayloadBuffer {
  std::vector<uint8_t> bytes;

  bool isValid() const { return true; }

  bool reserve(DataSize_t length) {
    if (length > bytes.max_size() - bytes.size()) {
      return false;
    }
    bytes.reserve(bytes.size() + length);
    return true;
  }

  bool append(const uint8_t *data, DataSize_t length) {
    if (length == 0) {
      return true;
    }
    if (data == nullptr) {
      return false;
    }

    bytes.insert(bytes.end(), data, data + length);
    return true;
  }

  void append(const PayloadBuffer &other) {
    bytes.insert(bytes.end(), other.bytes.begin(), other.bytes.end());
  }

  DataSize_t spaceUsed() const { return static_cast<DataSize_t>(bytes.size()); }

  void reset() { bytes.clear(); }
};

} // namespace rtps

#endif // RTPS_PAYLOADBUFFER_H
