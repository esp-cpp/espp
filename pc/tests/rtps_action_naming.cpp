// Unit test for ROS 2 action name/type mangling (rtps::rpc). Expected strings are
// taken verbatim from a live rmw_fastrtps (ROS 2 Jazzy) Fibonacci action capture
// (RMI_AMI_DESIGN.md 3.4). Header-only.

#include <cstdio>
#include <string>

#include "rtps/rpc/action_naming.hpp"

namespace {
int failures = 0;
void check(const std::string &got, const std::string &want, const char *what) {
  if (got != want) {
    std::printf("FAIL %s: got \"%s\" want \"%s\"\n", what, got.c_str(), want.c_str());
    ++failures;
  } else {
    std::printf("ok   %s = \"%s\"\n", what, got.c_str());
  }
}
} // namespace

int main() {
  using namespace rtps::rpc;

  const std::string base = "example_interfaces::action::dds_::Fibonacci";

  // send_goal service: rq/fibonacci/_action/send_goalRequest, _SendGoal_Request_.
  check(service_request_topic(action_send_goal_service("/fibonacci")),
        "rq/fibonacci/_action/send_goalRequest", "send_goal_req_topic");
  check(service_reply_topic(action_send_goal_service("/fibonacci")),
        "rr/fibonacci/_action/send_goalReply", "send_goal_rep_topic");
  check(service_request_type(action_send_goal_type(base)),
        "example_interfaces::action::dds_::Fibonacci_SendGoal_Request_", "send_goal_req_type");
  check(service_response_type(action_send_goal_type(base)),
        "example_interfaces::action::dds_::Fibonacci_SendGoal_Response_", "send_goal_rep_type");

  // get_result service.
  check(service_request_topic(action_get_result_service("/fibonacci")),
        "rq/fibonacci/_action/get_resultRequest", "get_result_req_topic");
  check(service_request_type(action_get_result_type(base)),
        "example_interfaces::action::dds_::Fibonacci_GetResult_Request_", "get_result_req_type");

  // cancel_goal service (fixed action_msgs type).
  check(service_request_topic(action_cancel_goal_service("/fibonacci")),
        "rq/fibonacci/_action/cancel_goalRequest", "cancel_goal_req_topic");
  check(service_request_type(action_cancel_goal_type()),
        "action_msgs::srv::dds_::CancelGoal_Request_", "cancel_goal_req_type");

  // topics.
  check(action_feedback_topic("/fibonacci"), "rt/fibonacci/_action/feedback", "feedback_topic");
  check(action_status_topic("/fibonacci"), "rt/fibonacci/_action/status", "status_topic");
  check(action_feedback_type(base), "example_interfaces::action::dds_::Fibonacci_FeedbackMessage_",
        "feedback_type");
  check(action_status_type(), "action_msgs::msg::dds_::GoalStatusArray_", "status_type");

  if (failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d\n", failures);
  return 1;
}
