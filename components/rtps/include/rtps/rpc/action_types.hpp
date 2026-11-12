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

#ifndef RTPS_RPC_ACTION_TYPES_H
#define RTPS_RPC_ACTION_TYPES_H

// ---------------------------------------------------------------------------
// ROS 2 action envelope codec: wraps/unwraps the user's opaque goal/result/
// feedback CDR payloads in the action message envelopes (goal_id UUID, accepted
// flag, stamp, status). All layouts CDR_LE, confirmed against a live Fibonacci
// capture (RMI_AMI_DESIGN.md 3.4). Every payload here (in and out) is a full
// CDR message: a 4-byte encapsulation header {0x00,0x01,0x00,0x00} + body,
// exactly like the service request/reply payloads.
//
// The nested user payload (goal / result / feedback) is spliced in field-wise:
// we strip its 4-byte encapsulation header and place its body after the envelope
// prefix, under the envelope's single encapsulation header. This is byte-exact
// as long as the nested body starts at a CDR offset with matching alignment:
//   - goal follows UUID(16)  -> offset 16 (8-aligned): correct for any field.
//   - feedback follows UUID(16) -> offset 16 (8-aligned): correct for any field.
//   - result follows status(1)+pad(3) -> offset 4 (4-aligned): correct only
//     when EVERY field of the result is at most 4-byte aligned (int32/uint32/
//     float/bool/strings/arrays/nested of those). CDR aligns each field to the
//     stream origin, so a standalone-serialized result (its fields aligned
//     relative to its own offset 0) spliced in 4 bytes later has every int64 /
//     float64 / uint64 field - first or not - off by 4. Such a Result must be
//     serialized as part of the response ({status, result} as one message):
//     RtpsParticipant::ActionGoalHandle::terminate_with_response() takes that
//     pre-built response and ActionClient::send_goal_raw() hands it back; the
//     typed espp::ActionServer / ActionClient (rtps_action.hpp,
//     GetResultResponse<Result>) do exactly that.
// ---------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace rtps {
namespace rpc {

// unique_identifier_msgs/UUID: 16 raw bytes.
using GoalUuid = std::array<uint8_t, 16>;

// GoalStatus.status values (action_msgs/msg/GoalStatus).
enum class GoalStatus : int8_t {
  UNKNOWN = 0,
  ACCEPTED = 1,
  EXECUTING = 2,
  CANCELING = 3,
  SUCCEEDED = 4,
  CANCELED = 5,
  ABORTED = 6,
};

namespace detail {
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};
inline bool has_encap(std::span<const uint8_t> b) { return b.size() >= 4; }
inline void put_u32(std::vector<uint8_t> &v, uint32_t x) {
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));
  }
}
inline uint32_t get_u32(std::span<const uint8_t> b, size_t off) {
  return static_cast<uint32_t>(b[off]) | (static_cast<uint32_t>(b[off + 1]) << 8) |
         (static_cast<uint32_t>(b[off + 2]) << 16) | (static_cast<uint32_t>(b[off + 3]) << 24);
}
} // namespace detail

// --- SendGoal_Request = { goal_id: UUID(16), goal } ---
inline std::vector<uint8_t> wrap_send_goal_request(const GoalUuid &id,
                                                   std::span<const uint8_t> goal_cdr) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.insert(v.end(), id.begin(), id.end());
  if (detail::has_encap(goal_cdr)) {
    v.insert(v.end(), goal_cdr.begin() + 4, goal_cdr.end()); // strip nested encap
  }
  return v;
}
// Returns false if too short. goal_cdr_out gets a re-encapsulated nested payload.
inline bool unwrap_send_goal_request(std::span<const uint8_t> msg, GoalUuid &id_out,
                                     std::vector<uint8_t> &goal_cdr_out) {
  if (msg.size() < 4 + 16) {
    return false;
  }
  std::memcpy(id_out.data(), msg.data() + 4, 16);
  goal_cdr_out.assign(detail::kEncap, detail::kEncap + 4);
  goal_cdr_out.insert(goal_cdr_out.end(), msg.begin() + 4 + 16, msg.end());
  return true;
}

// --- SendGoal_Response = { accepted: bool(1)+pad(3), stamp{sec:i32,nsec:u32} } ---
inline std::vector<uint8_t> make_send_goal_response(bool accepted, int32_t sec = 0,
                                                    uint32_t nsec = 0) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.push_back(accepted ? 1 : 0);
  v.push_back(0);
  v.push_back(0);
  v.push_back(0);
  detail::put_u32(v, static_cast<uint32_t>(sec));
  detail::put_u32(v, nsec);
  return v;
}
inline bool parse_send_goal_response(std::span<const uint8_t> msg, bool &accepted_out) {
  if (msg.size() < 4 + 4) {
    return false;
  }
  accepted_out = (msg[4] != 0);
  return true;
}

// --- GetResult_Request = { goal_id: UUID(16) } ---
inline std::vector<uint8_t> make_get_result_request(const GoalUuid &id) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.insert(v.end(), id.begin(), id.end());
  return v;
}
inline bool parse_get_result_request(std::span<const uint8_t> msg, GoalUuid &id_out) {
  if (msg.size() < 4 + 16) {
    return false;
  }
  std::memcpy(id_out.data(), msg.data() + 4, 16);
  return true;
}

// --- GetResult_Response = { status: i8(1)+pad(3), result } ---
inline std::vector<uint8_t> wrap_get_result_response(GoalStatus status,
                                                     std::span<const uint8_t> result_cdr) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.push_back(static_cast<uint8_t>(status));
  v.push_back(0);
  v.push_back(0);
  v.push_back(0);
  if (detail::has_encap(result_cdr)) {
    v.insert(v.end(), result_cdr.begin() + 4, result_cdr.end());
  }
  return v;
}
inline bool unwrap_get_result_response(std::span<const uint8_t> msg, GoalStatus &status_out,
                                       std::vector<uint8_t> &result_cdr_out) {
  if (msg.size() < 4 + 4) {
    return false;
  }
  status_out = static_cast<GoalStatus>(static_cast<int8_t>(msg[4]));
  result_cdr_out.assign(detail::kEncap, detail::kEncap + 4);
  result_cdr_out.insert(result_cdr_out.end(), msg.begin() + 4 + 4, msg.end());
  return true;
}

// --- FeedbackMessage = { goal_id: UUID(16), feedback } ---
inline std::vector<uint8_t> wrap_feedback(const GoalUuid &id,
                                          std::span<const uint8_t> feedback_cdr) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.insert(v.end(), id.begin(), id.end());
  if (detail::has_encap(feedback_cdr)) {
    v.insert(v.end(), feedback_cdr.begin() + 4, feedback_cdr.end());
  }
  return v;
}
inline bool unwrap_feedback(std::span<const uint8_t> msg, GoalUuid &id_out,
                            std::vector<uint8_t> &feedback_cdr_out) {
  if (msg.size() < 4 + 16) {
    return false;
  }
  std::memcpy(id_out.data(), msg.data() + 4, 16);
  feedback_cdr_out.assign(detail::kEncap, detail::kEncap + 4);
  feedback_cdr_out.insert(feedback_cdr_out.end(), msg.begin() + 4 + 16, msg.end());
  return true;
}

// --- CancelGoal (action_msgs/srv/CancelGoal) ---
// Request  = { goal_info{ goal_id: UUID(16), stamp{sec:i32, nsec:u32} } }; a zero
//            goal id addresses every goal.
// Response = { return_code: i8(1)+pad(3), goals_canceling: GoalInfo[] } with
//            GoalInfo = { goal_id: UUID(16), stamp{sec, nsec} } (24 bytes each).
constexpr int8_t CANCEL_ERROR_NONE = 0;
constexpr int8_t CANCEL_ERROR_REJECTED = 1;
constexpr int8_t CANCEL_ERROR_UNKNOWN_GOAL_ID = 2;
constexpr int8_t CANCEL_ERROR_GOAL_TERMINATED = 3;

inline std::vector<uint8_t> make_cancel_goal_request(const GoalUuid &id, int32_t sec = 0,
                                                     uint32_t nsec = 0) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.insert(v.end(), id.begin(), id.end());
  detail::put_u32(v, static_cast<uint32_t>(sec));
  detail::put_u32(v, nsec);
  return v;
}
inline bool parse_cancel_goal_request(std::span<const uint8_t> msg, GoalUuid &id_out,
                                      int32_t &sec_out, uint32_t &nsec_out) {
  if (msg.size() < 4 + 16 + 8) {
    return false;
  }
  std::memcpy(id_out.data(), msg.data() + 4, 16);
  sec_out = static_cast<int32_t>(detail::get_u32(msg, 20));
  nsec_out = detail::get_u32(msg, 24);
  return true;
}
inline bool parse_cancel_goal_request(std::span<const uint8_t> msg, GoalUuid &id_out) {
  int32_t sec = 0;
  uint32_t nsec = 0;
  return parse_cancel_goal_request(msg, id_out, sec, nsec);
}
inline std::vector<uint8_t> make_cancel_goal_response(int8_t return_code,
                                                      std::span<const GoalUuid> goals_canceling) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  v.push_back(static_cast<uint8_t>(return_code));
  v.push_back(0);
  v.push_back(0);
  v.push_back(0);
  detail::put_u32(v, static_cast<uint32_t>(goals_canceling.size()));
  for (const auto &id : goals_canceling) {
    v.insert(v.end(), id.begin(), id.end());
    detail::put_u32(v, 0); // stamp.sec: the cancel time is not tracked
    detail::put_u32(v, 0); // stamp.nanosec
  }
  return v;
}
inline bool parse_cancel_goal_response(std::span<const uint8_t> msg, int8_t &return_code_out,
                                       std::vector<GoalUuid> &goals_canceling_out) {
  if (msg.size() < 4 + 4 + 4) {
    return false;
  }
  return_code_out = static_cast<int8_t>(msg[4]);
  const uint32_t n = detail::get_u32(msg, 8);
  size_t off = 12;
  goals_canceling_out.clear();
  for (uint32_t i = 0; i < n; ++i) {
    if (off + 24 > msg.size()) {
      return false;
    }
    GoalUuid id{};
    std::memcpy(id.data(), msg.data() + off, 16);
    goals_canceling_out.push_back(id);
    off += 24;
  }
  return true;
}

// --- GoalStatusArray = status_list[]{ goal_id:UUID(16), stamp{sec,nsec}, status:i8+pad } ---
struct GoalStatusEntry {
  GoalUuid goal_id{};
  int32_t sec{0};
  uint32_t nsec{0};
  GoalStatus status{GoalStatus::UNKNOWN};
};
inline std::vector<uint8_t> make_goal_status_array(std::span<const GoalStatusEntry> entries) {
  std::vector<uint8_t> v(detail::kEncap, detail::kEncap + 4);
  detail::put_u32(v, static_cast<uint32_t>(entries.size()));
  for (const auto &e : entries) {
    v.insert(v.end(), e.goal_id.begin(), e.goal_id.end());
    detail::put_u32(v, static_cast<uint32_t>(e.sec));
    detail::put_u32(v, e.nsec);
    v.push_back(static_cast<uint8_t>(e.status));
    v.push_back(0);
    v.push_back(0);
    v.push_back(0);
  }
  return v;
}
inline bool parse_goal_status_array(std::span<const uint8_t> msg,
                                    std::vector<GoalStatusEntry> &out) {
  if (msg.size() < 4 + 4) {
    return false;
  }
  const uint32_t n = detail::get_u32(msg, 4);
  size_t off = 8;
  out.clear();
  for (uint32_t i = 0; i < n; ++i) {
    if (off + 16 + 4 + 4 + 4 > msg.size()) {
      return false;
    }
    GoalStatusEntry e;
    std::memcpy(e.goal_id.data(), msg.data() + off, 16);
    off += 16;
    e.sec = static_cast<int32_t>(detail::get_u32(msg, off));
    off += 4;
    e.nsec = detail::get_u32(msg, off);
    off += 4;
    e.status = static_cast<GoalStatus>(static_cast<int8_t>(msg[off]));
    off += 4; // status(1) + pad(3)
    out.push_back(e);
  }
  return true;
}

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_ACTION_TYPES_H
