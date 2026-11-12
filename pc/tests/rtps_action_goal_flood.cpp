// An action server bounds what it runs and what it keeps.
//
// Every accepted goal runs on its own std::thread, and nothing capped them:
// a client (or a retrying one) that submitted N goals spawned N threads, and
// on a device thread exhaustion aborts. A goal id the server already held
// silently replaced the in-flight goal and orphaned its pending get_result
// (rcl rejects duplicates). A finished goal whose result nobody fetched
// stayed in the server's map forever. Now ActionConfig::max_goals_in_flight
// (8 by default) rejects a goal beyond the cap before on_goal sees it,
// duplicate ids are rejected, and a finished goal is retired
// result_timeout after it terminated.
//
// Byte-level service clients on the action's own send_goal / cancel_goal /
// get_result services (what a ROS 2 client speaks):
//  1. 20 goals against max_goals_in_flight 3, all blocking until cancelled:
//     exactly 3 accepted, 17 rejected, no crash.
//  2. An accepted id sent again is rejected.
//  3. Cancel-all (a zero id): the 3 goals are listed, finish CANCELED, and
//     get_result says so right away.
//  4. With result_timeout 300 ms, get_result a second later is UNKNOWN, and
//     the server accepts goals again.
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <span>
#include <thread>
#include <vector>

#include "rtps/rpc/action_naming.hpp"
#include "rtps/rpc/action_types.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;
using Rtps = espp::RtpsParticipant;
namespace ract = rtps::rpc;

namespace {
#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

constexpr const char *kAction = "/flood";
constexpr const char *kType = "espp_test::action::dds_::Flood";
// Goal = Result = {} (one placeholder byte, as ROS 2 encodes an empty message).
const std::vector<uint8_t> kEmpty{0x00, 0x01, 0x00, 0x00, 0x00};

ract::GoalUuid make_id(uint8_t n) {
  ract::GoalUuid id{};
  std::mt19937 rng(12345u + n);
  for (auto &b : id) {
    b = static_cast<uint8_t>(rng());
  }
  id[0] = n;
  return id;
}

bool run() {
  Rtps server({.log_level = espp::Logger::Verbosity::ERROR});
  Rtps client({.log_level = espp::Logger::Verbosity::ERROR});
  CHECK(server.start() && client.start(), "start");

  std::atomic<int> executing{0};
  CHECK(server.add_action_server(
            {.action = kAction,
             .type_name = kType,
             .max_goals_in_flight = 3,
             .result_timeout = 300ms},
            [](const Rtps::GoalId &, std::span<const uint8_t>) { return true; },
            [&executing](Rtps::ActionGoalHandle h) {
              executing.fetch_add(1);
              while (!h.is_canceling()) {
                std::this_thread::sleep_for(5ms);
              }
              executing.fetch_sub(1);
              h.canceled(kEmpty);
            }),
        "add_action_server");

  auto send_goal = client.add_service_client({.service = ract::action_send_goal_service(kAction),
                                              .type_name = ract::action_send_goal_type(kType)});
  auto cancel = client.add_service_client({.service = ract::action_cancel_goal_service(kAction),
                                           .type_name = ract::action_cancel_goal_type()});
  auto get_result = client.add_service_client({.service = ract::action_get_result_service(kAction),
                                               .type_name = ract::action_get_result_type(kType)});
  CHECK(send_goal && cancel && get_result, "service clients");
  CHECK(send_goal->wait_for_server(10s) && cancel->wait_for_server(10s) &&
            get_result->wait_for_server(10s),
        "wait_for_server");

  const auto submit = [&](const ract::GoalUuid &id) -> int { // 1 accepted, 0 rejected, -1 error
    const auto reply = send_goal->call(ract::wrap_send_goal_request(id, kEmpty), 5s);
    bool accepted = false;
    if (!reply || !ract::parse_send_goal_response(*reply, accepted)) {
      return -1;
    }
    return accepted ? 1 : 0;
  };

  // 1. The flood.
  std::vector<ract::GoalUuid> ids;
  int accepted = 0;
  int rejected = 0;
  for (uint8_t n = 1; n <= 20; ++n) {
    ids.push_back(make_id(n));
    const int r = submit(ids.back());
    CHECK(r >= 0, "send_goal answered");
    accepted += r;
    rejected += r == 0 ? 1 : 0;
  }
  std::printf("20 goals: %d accepted, %d rejected, %d executing\n", accepted, rejected,
              executing.load());
  CHECK(accepted == 3 && rejected == 17, "exactly max_goals_in_flight accepted");

  // 2. A duplicate of an accepted id.
  CHECK(submit(ids.front()) == 0, "a duplicate goal id is rejected");

  // 3. Cancel all: the three are listed, finish CANCELED, get_result agrees.
  {
    const auto reply = cancel->call(ract::make_cancel_goal_request(ract::GoalUuid{}), 5s);
    int8_t code = -1;
    std::vector<ract::GoalUuid> canceling;
    CHECK(reply && ract::parse_cancel_goal_response(*reply, code, canceling), "cancel answered");
    CHECK(code == ract::CANCEL_ERROR_NONE && canceling.size() == 3,
          "cancel-all lists the three running goals");
  }
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (executing.load() != 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  CHECK(executing.load() == 0, "the goals wound down");
  {
    const auto reply = get_result->call(ract::make_get_result_request(ids.front()), 5s);
    ract::GoalStatus status{};
    std::vector<uint8_t> result;
    CHECK(reply && ract::unwrap_get_result_response(*reply, status, result), "get_result answered");
    CHECK(status == ract::GoalStatus::CANCELED, "get_result right after: CANCELED");
  }

  // 4. Past result_timeout the goal is retired; the server has room again.
  std::this_thread::sleep_for(1s);
  {
    const auto reply = get_result->call(ract::make_get_result_request(ids.front()), 5s);
    ract::GoalStatus status{};
    std::vector<uint8_t> result;
    CHECK(reply && ract::unwrap_get_result_response(*reply, status, result),
          "get_result answered again");
    CHECK(status == ract::GoalStatus::UNKNOWN, "a retired goal is UNKNOWN");
  }
  CHECK(submit(make_id(42)) == 1, "a new goal is accepted once the others are gone");
  {
    const auto reply = cancel->call(ract::make_cancel_goal_request(ract::GoalUuid{}), 5s);
    CHECK(reply.has_value(), "final cancel");
  }
  client.stop();
  server.stop();
  return true;
}
} // namespace

int main() {
  if (!run()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
