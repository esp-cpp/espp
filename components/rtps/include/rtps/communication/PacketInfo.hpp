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

// Copyright 2023 Apex.AI, Inc.
// All rights reserved.

#ifndef RTPS_PACKETINFO_H
#define RTPS_PACKETINFO_H

#include <array>
#include <vector>

#include "rtps/common/types.hpp"

namespace rtps {

struct PacketInfo {
  Ip4Port_t srcPort; // TODO Do we need that?
  std::array<uint8_t, 4> destAddr = {0, 0, 0, 0};
  Ip4Port_t destPort;
  std::vector<uint8_t> payload;

  void copyTriviallyCopyable(const PacketInfo &other) {
    this->srcPort = other.srcPort;
    this->destPort = other.destPort;
    this->destAddr = other.destAddr;
  }

  PacketInfo() = default;
  ~PacketInfo() = default;

  PacketInfo &operator=(const PacketInfo &other) = delete;

  PacketInfo &operator=(PacketInfo &&other) noexcept {
    copyTriviallyCopyable(other);
    this->payload = std::move(other.payload);
    return *this;
  }
};
} // namespace rtps

#endif // RTPS_PACKETINFO_H
