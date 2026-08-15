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

#ifndef RTPS_RPC_ACTION_NAMING_H
#define RTPS_RPC_ACTION_NAMING_H

// ---------------------------------------------------------------------------
// ROS 2 (rmw_fastrtps) action name/type mangling. An action maps onto 3 services
// (send_goal, cancel_goal, get_result) + 2 topics (feedback, status). Verified
// against a live rmw_fastrtps (ROS 2 Jazzy) Fibonacci action capture (see
// RMI_AMI_DESIGN.md 3.4):
//
//   action "/fibonacci", base type "example_interfaces::action::dds_::Fibonacci"
//     send_goal svc   base "fibonacci/_action/send_goal"   type "...Fibonacci_SendGoal"
//     cancel_goal svc base "fibonacci/_action/cancel_goal" type
//     "action_msgs::srv::dds_::CancelGoal" get_result svc  base "fibonacci/_action/get_result" type
//     "...Fibonacci_GetResult" feedback topic  "rt/fibonacci/_action/feedback"  type
//     "...Fibonacci_FeedbackMessage_" status topic    "rt/fibonacci/_action/status"    type
//     "action_msgs::msg::dds_::GoalStatusArray_"
//
// The service base names feed rpc::service_request_topic/service_request_type
// (etc.) to get the final rq/rr topics and _Request_/_Response_ types, reusing
// the service mangling exactly. Header-only, unit-testable.
// ---------------------------------------------------------------------------

#include "rtps/rpc/service_naming.hpp"

#include <string>
#include <string_view>

namespace rtps {
namespace rpc {

// --- service base names (feed to service_{request,reply}_topic / _type) ---

inline std::string action_send_goal_service(std::string_view action) {
  return strip_leading_slash(action) + "/_action/send_goal";
}
inline std::string action_cancel_goal_service(std::string_view action) {
  return strip_leading_slash(action) + "/_action/cancel_goal";
}
inline std::string action_get_result_service(std::string_view action) {
  return strip_leading_slash(action) + "/_action/get_result";
}

// --- topics ---

inline std::string action_feedback_topic(std::string_view action) {
  return "rt/" + strip_leading_slash(action) + "/_action/feedback";
}
inline std::string action_status_topic(std::string_view action) {
  return "rt/" + strip_leading_slash(action) + "/_action/status";
}

// --- service base types (feed to service_request_type / service_response_type) ---
// base = "<pkg>::action::dds_::<Action>", e.g. "..::action::dds_::Fibonacci".

inline std::string action_send_goal_type(std::string_view base) {
  return std::string(base) + "_SendGoal";
}
inline std::string action_get_result_type(std::string_view base) {
  return std::string(base) + "_GetResult";
}
inline std::string action_feedback_type(std::string_view base) {
  return std::string(base) + "_FeedbackMessage_";
}

// Fixed action_msgs types (not derived from the action's own type).
inline std::string action_cancel_goal_type() { return "action_msgs::srv::dds_::CancelGoal"; }
inline std::string action_status_type() { return "action_msgs::msg::dds_::GoalStatusArray_"; }

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_ACTION_NAMING_H
