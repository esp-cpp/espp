// Unit test for ROS 2 service name/type mangling (rtps::rpc). The expected
// strings are taken verbatim from a live rmw_fastrtps (ROS 2 Jazzy) AddTwoInts
// capture (see components/rtps/RMI_AMI_DESIGN.md 3.1/3.2). Header-only,
// no engine/runtime needed.

#include <cstdio>
#include <string>

#include "rtps/rpc/service_naming.hpp"

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

  const std::string base = "example_interfaces::srv::dds_::AddTwoInts";

  // Captured wire strings.
  check(service_request_topic("/add_two_ints"), "rq/add_two_intsRequest", "request_topic");
  check(service_reply_topic("/add_two_ints"), "rr/add_two_intsReply", "reply_topic");
  check(service_request_type(base), "example_interfaces::srv::dds_::AddTwoInts_Request_",
        "request_type");
  check(service_response_type(base), "example_interfaces::srv::dds_::AddTwoInts_Response_",
        "response_type");

  // Namespaced service keeps internal slashes; leading slash stripped once.
  check(service_request_topic("/ns/svc"), "rq/ns/svcRequest", "ns_request_topic");
  check(service_reply_topic("/ns/svc"), "rr/ns/svcReply", "ns_reply_topic");
  // No leading slash is accepted as-is.
  check(service_request_topic("svc"), "rq/svcRequest", "noslash_request_topic");

  if (failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d\n", failures);
  return 1;
}
