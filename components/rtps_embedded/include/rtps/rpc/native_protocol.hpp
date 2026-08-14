/*
The MIT License
Copyright (c) 2026 ATDev
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

This file is part of the espp embeddedRTPS port.
*/

#ifndef RTPS_RPC_NATIVE_PROTOCOL_H
#define RTPS_RPC_NATIVE_PROTOCOL_H

// ---------------------------------------------------------------------------
// Native (espp<->espp) request/reply protocol - Track B in RMI_AMI_DESIGN.md.
// Deliberately NOT ROS 2-compatible: it trades interop for simplicity + a small
// footprint. Correlation is an in-band 20-byte header prepended to the payload,
// so it needs no inline-QoS engine support and rides plain reliable pub/sub.
//
//   offset 0   client_prefix : 12 bytes  (the requesting participant's GUID
//                                          prefix - the correlation key; the
//                                          server echoes it on the reply)
//   offset 12  request_id     : uint32 LE (client-monotonic)
//   offset 16  op             : uint8     (REQUEST=0, REPLY=1)
//   offset 17  flags          : uint8     (reserved; 0)
//   offset 18  reserved       : uint16    (0)
//   offset 20  <payload>                  (the CDR-encapsulated user message)
//
// A request is broadcast on es_rq/<service>; the server echoes {client_prefix,
// request_id} on the reply, broadcast on es_rr/<service>. Each client accepts
// only replies whose client_prefix == its own prefix and matches request_id
// against its pending table (client-side filtering, same shape as the ROS path
// but with the key in-band instead of as related_sample_identity inline QoS).
//
// Single-server assumption (v1): a broadcast request reaches every matched
// server, so N servers on one topic would each reply. Run one server per service.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace rtps {
namespace rpc {

constexpr std::size_t NATIVE_HEADER_SIZE = 20;

enum class NativeOp : uint8_t {
  REQUEST = 0,
  REPLY = 1,
};

struct NativeHeader {
  std::array<uint8_t, 12> client_prefix{};
  uint32_t request_id{0};
  NativeOp op{NativeOp::REQUEST};
  uint8_t flags{0};
};

// Prepend the native header to a payload, producing the transported frame.
inline std::vector<uint8_t> native_encode(const NativeHeader &h, std::span<const uint8_t> payload) {
  std::vector<uint8_t> v;
  v.reserve(NATIVE_HEADER_SIZE + payload.size());
  v.insert(v.end(), h.client_prefix.begin(), h.client_prefix.end());
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((h.request_id >> (8 * i)) & 0xFF));
  }
  v.push_back(static_cast<uint8_t>(h.op));
  v.push_back(h.flags);
  v.push_back(0); // reserved
  v.push_back(0);
  v.insert(v.end(), payload.begin(), payload.end());
  return v;
}

// Parse a frame: fill the header and return the payload span (into `frame`).
// Returns false if the frame is too short to hold the header.
inline bool native_decode(std::span<const uint8_t> frame, NativeHeader &h_out,
                          std::span<const uint8_t> &payload_out) {
  if (frame.size() < NATIVE_HEADER_SIZE) {
    return false;
  }
  std::memcpy(h_out.client_prefix.data(), frame.data(), 12);
  h_out.request_id = static_cast<uint32_t>(frame[12]) | (static_cast<uint32_t>(frame[13]) << 8) |
                     (static_cast<uint32_t>(frame[14]) << 16) |
                     (static_cast<uint32_t>(frame[15]) << 24);
  h_out.op = static_cast<NativeOp>(frame[16]);
  h_out.flags = frame[17];
  payload_out = frame.subspan(NATIVE_HEADER_SIZE);
  return true;
}

// Native topics - a distinct prefix so they never alias the ROS rq/rr topics.
inline std::string native_request_topic(std::string_view service) {
  std::string s(service);
  if (!s.empty() && s.front() == '/') {
    s.erase(0, 1);
  }
  return "es_rq/" + s;
}
inline std::string native_reply_topic(std::string_view service) {
  std::string s(service);
  if (!s.empty() && s.front() == '/') {
    s.erase(0, 1);
  }
  return "es_rr/" + s;
}

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_NATIVE_PROTOCOL_H
