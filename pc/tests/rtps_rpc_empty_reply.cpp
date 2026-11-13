// A service handler that returns nothing sends nothing.
//
// A handler returning an empty vector (the typed wrapper on a request it
// could not parse, a Python handler that raised) was sent as a 0-byte DATA:
// on the peer a deserialization error, not a timeout, and nothing logged on
// either side. ServiceResponder::reply() now refuses an empty response with
// an error log and sends nothing, so the client times out as it would for a
// handler that never answered; replies dropped because the reply writer's
// history is full are logged too.
//
//  1. A handler returning {}: the client's call() times out (nullopt) - no
//     reply ever arrives, malformed or otherwise.
//  2. A second service in the same process answers normally, so the empty
//     reply poisoned nothing.
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;
using Rtps = espp::RtpsParticipant;

namespace {
const std::vector<uint8_t> kRequest{0x00, 0x01, 0x00, 0x00, 1, 0, 0, 0, 0, 0,
                                    0,    0,    2,    0,    0, 0, 0, 0, 0, 0};
constexpr const char *kType = "example_interfaces::srv::dds_::AddTwoInts";
} // namespace

int main() {
  Rtps server({.log_level = espp::Logger::Verbosity::WARN});
  Rtps client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  std::atomic<int> empty_calls{0};
  if (!server.add_service_server({.service = "/empty", .type_name = kType},
                                 [&empty_calls](std::span<const uint8_t>) {
                                   empty_calls.fetch_add(1);
                                   return std::vector<uint8_t>{};
                                 }) ||
      !server.add_service_server({.service = "/ok", .type_name = kType},
                                 [](std::span<const uint8_t> req) {
                                   return std::vector<uint8_t>(req.begin(), req.end());
                                 })) {
    std::printf("FAIL: add_service_server\n");
    return 1;
  }
  auto empty = client.add_service_client({.service = "/empty", .type_name = kType});
  auto ok = client.add_service_client({.service = "/ok", .type_name = kType});
  if (!empty || !ok || !empty->wait_for_server(10s) || !ok->wait_for_server(10s)) {
    std::printf("FAIL: clients / wait_for_server\n");
    return 1;
  }

  // 1. Nothing comes back from the empty handler, within or after the timeout.
  const auto t0 = std::chrono::steady_clock::now();
  const auto reply = empty->call(kRequest, 300ms);
  const auto waited =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
  if (reply.has_value()) {
    std::printf("FAIL: a reply (%zu bytes) arrived from a handler that returned nothing\n",
                reply->size());
    return 1;
  }
  if (waited < 250ms) {
    std::printf("FAIL: call() gave up early (%lld ms): something answered\n",
                static_cast<long long>(waited.count()));
    return 1;
  }
  if (empty_calls.load() == 0) {
    std::printf("FAIL: the empty handler never ran\n");
    return 1;
  }
  std::printf("empty handler: call() timed out after %lld ms, as it should\n",
              static_cast<long long>(waited.count()));

  // 2. The other service still answers.
  const auto echoed = ok->call(kRequest, 5s);
  if (!echoed || *echoed != kRequest) {
    std::printf("FAIL: the echo service did not answer correctly\n");
    return 1;
  }
  client.stop();
  server.stop();
  std::printf("PASS\n");
  return 0;
}
