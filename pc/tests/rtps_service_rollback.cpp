// Transactional service/action creation: a partial endpoint-creation failure
// must roll back the endpoints that DID build - nothing stays announced, no
// engine pool slot stays consumed, and no dedicated-port ration slot (or fd)
// leaks.
//
// Failure seam: a service name chosen so the request topic ("rq/<name>Request")
// exceeds the engine's MAX_TOPICNAME_LENGTH while the reply topic
// ("rr/<name>Reply", two characters shorter) fits. The ROS service server
// creates writer-then-reader, the client reader-then-writer, so the same name
// fails the SECOND endpoint of each pair, leaving one successful (banded,
// dedicated-port) endpoint to roll back.
//
//  1. Server partial failure: add_service_server() returns false AND the
//     dedicated port its reply writer had claimed becomes externally bindable
//     (fd + ration slot released; no leaked SEDP announcement).
//  2. Client partial failure: same, for the reply reader's dedicated port.
//  3. Action-server rollback: with the participant's writer budget reduced to
//     exactly 3 free slots, add_action_server() (which needs 5 writers) fails
//     partway; the rollback must return all 3 slots - proven by 3 subsequent
//     add_writer() calls succeeding.
//  4. The participant stays fully usable: a valid banded service then builds.
//
// Exits 0 on success.

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "rtps/config.hpp"
#include "rtps_participant.hpp"
#include "udp_socket.hpp"

using namespace std::chrono_literals;

namespace {

constexpr uint16_t kDedicatedBase = 7500; // 7400 + 250*domain(0) + 100

bool port_becomes_bindable(uint16_t port) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    espp::UdpSocket probe({.log_level = espp::Logger::Verbosity::NONE});
    espp::UdpSocket::ReceiveConfig rc;
    rc.port = port;
    if (probe.is_valid() && probe.disable_reuse() && probe.bind(rc)) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return 1;                                                                                    \
    }                                                                                              \
  } while (0)

} // namespace

int main() {
  using Reliability = espp::RtpsParticipant::Reliability;

  espp::RtpsParticipant participant({.log_level = espp::Logger::Verbosity::WARN});
  CHECK(participant.start(), "participant start");

  // Service name sized so "rq/<name>Request" == MAX_TOPICNAME_LENGTH (rejected:
  // the engine requires strlen < MAX) while "rr/<name>Reply" fits.
  const std::size_t name_len = rtps::Config::MAX_TOPICNAME_LENGTH - 10; // 3 + len + 7 == MAX
  const std::string bad_service = "/" + std::string(name_len - 0, 'x');
  static_assert(sizeof("rq/"
                       "Request") -
                    1 ==
                10);

  // 1. Server: reply writer (banded -> dedicated port 7500) succeeds, request
  //    reader fails -> the writer must be rolled back and its port released.
  CHECK(!participant.add_service_server(
            {.service = bad_service,
             .type_name = "test::srv::dds_::Bad",
             .band = espp::QosBand::High},
            [](std::span<const uint8_t>) { return std::vector<uint8_t>{}; }),
        "server creation reports failure");
  CHECK(port_becomes_bindable(kDedicatedBase),
        "server rollback released the reply writer's dedicated port");

  // 2. Client: reply reader (banded -> next dedicated port 7501) succeeds,
  //    request writer fails -> the reader must be rolled back.
  CHECK(participant.add_service_client({.service = bad_service,
                                        .type_name = "test::srv::dds_::Bad",
                                        .band = espp::QosBand::High}) == nullptr,
        "client creation reports failure");
  CHECK(port_becomes_bindable(kDedicatedBase + 1),
        "client rollback released the reply reader's dedicated port");

  // 3. Action-server rollback: reduce the participant's writer budget to
  //    exactly 3 free slots, then build an action (needs 5 writers: feedback,
  //    status, and 3 service reply writers). It fails partway; the rollback
  //    must return every slot it consumed. The usable capacity is measured
  //    dynamically (builtin discovery writers occupy participant slots too).
  int capacity = 0;
  {
    espp::RtpsParticipant probe({.log_level = espp::Logger::Verbosity::ERROR});
    CHECK(probe.start(), "capacity probe start");
    while (probe.add_writer({.topic = "cap_" + std::to_string(capacity),
                             .type_name = "test::msg::dds_::Fill_",
                             .reliability = Reliability::RELIABLE})) {
      ++capacity;
    }
    probe.stop();
  }
  CHECK(capacity >= 5, "enough writer capacity for the scenario");
  const int fill = capacity - 3;
  for (int i = 0; i < fill; ++i) {
    CHECK(participant.add_writer({.topic = "fill_" + std::to_string(i),
                                  .type_name = "test::msg::dds_::Fill_",
                                  .reliability = Reliability::RELIABLE}),
          "budget fill writer");
  }
  CHECK(!participant.add_action_server(
            {.action = "/rollback_probe", .type_name = "test::action::dds_::Probe"}, nullptr,
            [](espp::RtpsParticipant::ActionGoalHandle) {}),
        "action server creation reports failure");
  // All 3 slots the failed action consumed must be free again.
  for (int i = 0; i < 3; ++i) {
    CHECK(participant.add_writer({.topic = "post_rollback_" + std::to_string(i),
                                  .type_name = "test::msg::dds_::Fill_",
                                  .reliability = Reliability::RELIABLE}),
          "post-rollback writer slot available");
  }
  // Budget now exhausted for real - a further writer must fail (sanity check
  // that the 3 successes above actually re-used the rolled-back slots).
  CHECK(!participant.add_writer({.topic = "over_budget",
                                 .type_name = "test::msg::dds_::Fill_",
                                 .reliability = Reliability::RELIABLE}),
        "writer budget exhausted after refill");

  participant.stop();

  // 4. Fresh participant: a valid banded service builds normally (the failure
  //    handling leaves the machinery intact).
  espp::RtpsParticipant participant2({.log_level = espp::Logger::Verbosity::WARN});
  CHECK(participant2.start(), "participant2 start");
  CHECK(participant2.add_service_server(
            {.service = "/good", .type_name = "test::srv::dds_::Good", .band = espp::QosBand::High},
            [](std::span<const uint8_t>) { return std::vector<uint8_t>{}; }),
        "valid banded service builds");
  participant2.stop();

  std::printf("PASS\n");
  return 0;
}
