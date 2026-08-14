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
inline std::string native_strip_slash(std::string_view s) {
  std::string r(s);
  if (!r.empty() && r.front() == '/') {
    r.erase(0, 1);
  }
  return r;
}
inline std::string native_request_topic(std::string_view service) {
  return "es_rq/" + native_strip_slash(service);
}
inline std::string native_reply_topic(std::string_view service) {
  return "es_rr/" + native_strip_slash(service);
}

// ---------------------------------------------------------------------------
// Native action (Track B, lean AMI): collapses ROS's 3 services + 2 topics to a
// send_goal native request/reply (-> {accepted, goal_handle}), a small cancel
// native request/reply (-> {accepted}), and ONE feedback topic. No UUIDs (a
// uint32 goal_handle), no separate get_result / status - the terminal result
// rides the feedback stream as a SUCCEEDED/ABORTED/CANCELED message. ~4
// endpoints/pair vs ROS's ~10. See RMI_AMI_DESIGN.md 4.3.
// ---------------------------------------------------------------------------

// Reuse the ROS GoalStatus values for conceptual parity (see action_types.hpp,
// but kept independent here so the native path has no ROS-envelope dependency).
enum class NativeGoalStatus : uint8_t {
  ACCEPTED = 1,
  EXECUTING = 2,
  SUCCEEDED = 4,
  CANCELED = 5,
  ABORTED = 6,
};

inline std::string native_goal_service(std::string_view action) {
  return native_strip_slash(action) + "/goal";
}
inline std::string native_cancel_service(std::string_view action) {
  return native_strip_slash(action) + "/cancel";
}
inline std::string native_feedback_topic(std::string_view action) {
  return "es_fb/" + native_strip_slash(action);
}

// cancel request = { goal_handle:uint32 } (after encap).
inline std::vector<uint8_t> native_make_cancel_request(uint32_t goal_handle) {
  std::vector<uint8_t> v{0x00, 0x01, 0x00, 0x00};
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((goal_handle >> (8 * i)) & 0xFF));
  }
  return v;
}
inline bool native_parse_cancel_request(std::span<const uint8_t> msg, uint32_t &goal_handle_out) {
  if (msg.size() < 4 + 4) {
    return false;
  }
  goal_handle_out = static_cast<uint32_t>(msg[4]) | (static_cast<uint32_t>(msg[5]) << 8) |
                    (static_cast<uint32_t>(msg[6]) << 16) | (static_cast<uint32_t>(msg[7]) << 24);
  return true;
}
// cancel reply = { accepted:uint8 + pad(3) } (after encap).
inline std::vector<uint8_t> native_make_cancel_reply(bool accepted) {
  return {0x00, 0x01, 0x00, 0x00, static_cast<uint8_t>(accepted ? 1 : 0), 0, 0, 0};
}
inline bool native_parse_cancel_reply(std::span<const uint8_t> msg, bool &accepted_out) {
  if (msg.size() < 4 + 4) {
    return false;
  }
  accepted_out = (msg[4] != 0);
  return true;
}

// send_goal reply = { accepted:uint8 + pad(3), goal_handle:uint32 } (after encap).
inline std::vector<uint8_t> native_make_goal_reply(bool accepted, uint32_t goal_handle) {
  std::vector<uint8_t> v{0x00, 0x01, 0x00, 0x00, static_cast<uint8_t>(accepted ? 1 : 0), 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((goal_handle >> (8 * i)) & 0xFF));
  }
  return v;
}
inline bool native_parse_goal_reply(std::span<const uint8_t> msg, bool &accepted_out,
                                    uint32_t &goal_handle_out) {
  if (msg.size() < 4 + 8) {
    return false;
  }
  accepted_out = (msg[4] != 0);
  goal_handle_out = static_cast<uint32_t>(msg[8]) | (static_cast<uint32_t>(msg[9]) << 8) |
                    (static_cast<uint32_t>(msg[10]) << 16) | (static_cast<uint32_t>(msg[11]) << 24);
  return true;
}

// feedback/result msg = { goal_handle:uint32, status:uint8 + pad(3), payload } (after encap).
inline std::vector<uint8_t> native_make_feedback(uint32_t goal_handle, NativeGoalStatus status,
                                                 std::span<const uint8_t> payload) {
  std::vector<uint8_t> v{0x00, 0x01, 0x00, 0x00};
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((goal_handle >> (8 * i)) & 0xFF));
  }
  v.push_back(static_cast<uint8_t>(status));
  v.push_back(0);
  v.push_back(0);
  v.push_back(0);
  if (payload.size() >= 4) {
    v.insert(v.end(), payload.begin() + 4, payload.end()); // splice past the payload's encap
  }
  return v;
}
inline bool native_parse_feedback(std::span<const uint8_t> msg, uint32_t &goal_handle_out,
                                  NativeGoalStatus &status_out, std::vector<uint8_t> &payload_out) {
  if (msg.size() < 4 + 8) {
    return false;
  }
  goal_handle_out = static_cast<uint32_t>(msg[4]) | (static_cast<uint32_t>(msg[5]) << 8) |
                    (static_cast<uint32_t>(msg[6]) << 16) | (static_cast<uint32_t>(msg[7]) << 24);
  status_out = static_cast<NativeGoalStatus>(msg[8]);
  payload_out.assign({0x00, 0x01, 0x00, 0x00});
  payload_out.insert(payload_out.end(), msg.begin() + 12, msg.end());
  return true;
}

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_NATIVE_PROTOCOL_H
