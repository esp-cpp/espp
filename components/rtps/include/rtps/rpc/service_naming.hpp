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

#ifndef RTPS_RPC_SERVICE_NAMING_H
#define RTPS_RPC_SERVICE_NAMING_H

// ---------------------------------------------------------------------------
// ROS 2 (rmw_fastrtps) service name/type mangling.
//
// A ROS 2 service maps onto a pair of DDS topics + types. Verified against a
// live rmw_fastrtps (ROS 2 Jazzy) AddTwoInts capture (see RMI_AMI_DESIGN.md 3.1):
//
//   service "/add_two_ints", base type "example_interfaces::srv::dds_::AddTwoInts"
//     request  topic "rq/add_two_intsRequest"  type "...AddTwoInts_Request_"
//     reply    topic "rr/add_two_intsReply"    type "...AddTwoInts_Response_"
//
// Topic rule: strip a single leading '/', prefix "rq"/"rr", suffix
// "Request"/"Reply". Internal slashes (namespaces) are preserved
// ("/ns/svc" -> "rq/ns/svcRequest"). Type rule: append "_Request_"/"_Response_"
// to the base DDS type name (already in "pkg::srv::dds_::Name" form, matching how
// the pub/sub facade takes "std_msgs::msg::dds_::UInt32_").
//
// Header-only, no engine dependency, so it is unit-testable on the host.
// ---------------------------------------------------------------------------

#include <string>
#include <string_view>

namespace rtps {
namespace rpc {

// Strip a single leading '/', if present. "/a/b" -> "a/b", "a" -> "a".
inline std::string strip_leading_slash(std::string_view service) {
  if (!service.empty() && service.front() == '/') {
    service.remove_prefix(1);
  }
  return std::string(service);
}

// "/add_two_ints" -> "rq/add_two_intsRequest"
inline std::string service_request_topic(std::string_view service) {
  return "rq/" + strip_leading_slash(service) + "Request";
}

// "/add_two_ints" -> "rr/add_two_intsReply"
inline std::string service_reply_topic(std::string_view service) {
  return "rr/" + strip_leading_slash(service) + "Reply";
}

// "example_interfaces::srv::dds_::AddTwoInts" -> "..._Request_"
inline std::string service_request_type(std::string_view base_type) {
  return std::string(base_type) + "_Request_";
}

// "example_interfaces::srv::dds_::AddTwoInts" -> "..._Response_"
inline std::string service_response_type(std::string_view base_type) {
  return std::string(base_type) + "_Response_";
}

} // namespace rpc
} // namespace rtps

#endif // RTPS_RPC_SERVICE_NAMING_H
