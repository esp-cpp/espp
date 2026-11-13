// An action server runs its goals on a worker pool it owns.
//
// Every accepted goal used to get a std::thread of its own (3 KB of internal
// RAM on ESP, created and destroyed per goal, no say over its stack, priority
// or core). Now ActionConfig::execute_workers tasks, configured by
// ActionConfig::execute_task, are created with the server and run accepted
// goals in acceptance order at the action's band; a goal waits for a free
// worker (status ACCEPTED), then runs (EXECUTING).
//
// Byte-level service clients on the action's send_goal / cancel_goal /
// get_result services and a reader of its status topic:
//  1. 4 goals against 2 workers (max_goals_in_flight 4), all blocking until
//     cancelled: all 4 accepted, exactly 2 execute at once, the other 2 are
//     reported ACCEPTED while they wait;
//  2. cancel-all: the running goals finish CANCELED, the waiting goals get
//     their turn, see the cancel and finish CANCELED too; every get_result
//     says CANCELED;
//  3. a server with 1 worker (the default) runs goals one after another.
//
// Exits 0 on success, 1 on failure.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
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

constexpr const char *kAction = "/pool";
constexpr const char *kType = "espp_test::action::dds_::Pool";
// Goal = Result = {} (one placeholder byte, as ROS 2 encodes an empty message).
const std::vector<uint8_t> kEmpty{0x00, 0x01, 0x00, 0x00, 0x00};

ract::GoalUuid make_id(uint8_t n) {
  ract::GoalUuid id{};
  id.fill(0x5a);
  id[0] = n;
  return id;
}

// The latest status the server published for each goal id (the server
// publishes one goal per GoalStatusArray message; merge them by id).
struct StatusBoard {
  std::mutex m;
  std::vector<ract::GoalStatusEntry> entries;
  void merge(const std::vector<ract::GoalStatusEntry> &update) {
    std::lock_guard<std::mutex> lock(m);
    for (const auto &u : update) {
      auto it = std::find_if(entries.begin(), entries.end(), [&u](const ract::GoalStatusEntry &e) {
        return e.goal_id == u.goal_id;
      });
      if (it == entries.end()) {
        entries.push_back(u);
      } else {
        *it = u;
      }
    }
  }
  ract::GoalStatus status_of(const ract::GoalUuid &id) {
    std::lock_guard<std::mutex> lock(m);
    const auto it = std::find_if(entries.begin(), entries.end(),
                                 [&id](const ract::GoalStatusEntry &e) { return e.goal_id == id; });
    return it == entries.end() ? ract::GoalStatus::UNKNOWN : it->status;
  }
};

bool wait_until(const std::function<bool()> &pred, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

bool run() {
  // Everything the callbacks touch is declared BEFORE the participants, so a
  // failed CHECK (which returns from here) destroys the participants - and
  // with them every callback - first.
  std::atomic<int> executing{0};
  std::atomic<int> peak{0};
  std::atomic<int> finished{0};
  StatusBoard board;
  std::atomic<int> serial_running{0};
  std::atomic<int> serial_peak{0};
  std::atomic<int> serial_done{0};

  Rtps server({.log_level = espp::Logger::Verbosity::ERROR});
  Rtps client({.log_level = espp::Logger::Verbosity::ERROR});
  CHECK(server.start() && client.start(), "start");

  const auto execute = [&](Rtps::ActionGoalHandle h) {
    const int now = executing.fetch_add(1) + 1;
    int seen = peak.load();
    while (now > seen && !peak.compare_exchange_weak(seen, now)) {
    }
    while (!h.is_canceling()) {
      std::this_thread::sleep_for(5ms);
    }
    executing.fetch_sub(1);
    h.canceled(kEmpty);
    finished.fetch_add(1);
  };
  CHECK(server.add_action_server(
            {.action = kAction,
             .type_name = kType,
             .max_goals_in_flight = 4,
             .execute_workers = 2,
             .execute_task = {.name = "pool_exec", .stack_size_bytes = 8192},
             .result_timeout = 2s},
            [](const Rtps::GoalId &, std::span<const uint8_t>) { return true; }, execute),
        "add_action_server");

  CHECK(client.add_reader({.topic = ract::action_status_topic(kAction),
                           .type_name = ract::action_status_type(),
                           .reliability = Rtps::Reliability::RELIABLE,
                           .on_sample =
                               [&board](std::span<const uint8_t> msg) {
                                 std::vector<ract::GoalStatusEntry> entries;
                                 if (ract::parse_goal_status_array(msg, entries)) {
                                   board.merge(entries);
                                 }
                               }}),
        "status reader");

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

  // 1. Four goals, two workers.
  std::vector<ract::GoalUuid> ids;
  for (uint8_t n = 1; n <= 4; ++n) {
    ids.push_back(make_id(n));
    CHECK(submit(ids.back()) == 1, "the goal is accepted");
  }
  CHECK(wait_until([&] { return executing.load() == 2; }, 2s), "two goals execute");
  // The invariant, watched for a while rather than sampled once: two goals
  // run, never a third (the other two are waiting for a worker).
  {
    const auto until = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < until) {
      CHECK(executing.load() == 2 && peak.load() == 2, "no more than two goals execute at once");
      std::this_thread::sleep_for(5ms);
    }
  }
  CHECK(wait_until(
            [&] {
              return board.status_of(ids[2]) == ract::GoalStatus::ACCEPTED &&
                     board.status_of(ids[3]) == ract::GoalStatus::ACCEPTED;
            },
            2s),
        "the waiting goals are reported ACCEPTED");
  CHECK(board.status_of(ids[0]) == ract::GoalStatus::EXECUTING &&
            board.status_of(ids[1]) == ract::GoalStatus::EXECUTING,
        "the running goals are reported EXECUTING");

  // 2. Cancel all: the running pair winds down, the waiting pair gets its turn.
  {
    const auto reply = cancel->call(ract::make_cancel_goal_request(ract::GoalUuid{}), 5s);
    int8_t code = -1;
    std::vector<ract::GoalUuid> canceling;
    CHECK(reply && ract::parse_cancel_goal_response(*reply, code, canceling), "cancel answered");
    CHECK(code == ract::CANCEL_ERROR_NONE && canceling.size() == 4, "cancel-all lists all four");
  }
  CHECK(wait_until([&] { return finished.load() == 4; }, 5s), "every goal ran and finished");
  CHECK(peak.load() == 2, "still no more than two at once");
  for (const auto &id : ids) {
    const auto reply = get_result->call(ract::make_get_result_request(id), 5s);
    ract::GoalStatus status{};
    std::vector<uint8_t> result;
    CHECK(reply && ract::unwrap_get_result_response(*reply, status, result), "get_result answered");
    CHECK(status == ract::GoalStatus::CANCELED, "get_result: CANCELED");
  }

  // 3. The default: one worker, goals one after another.
  constexpr const char *kSerial = "/pool_serial";
  CHECK(server.add_action_server(
            {.action = kSerial, .type_name = kType, .result_timeout = 2s},
            [](const Rtps::GoalId &, std::span<const uint8_t>) { return true; },
            [&](Rtps::ActionGoalHandle h) {
              const int now = serial_running.fetch_add(1) + 1;
              int seen = serial_peak.load();
              while (now > seen && !serial_peak.compare_exchange_weak(seen, now)) {
              }
              std::this_thread::sleep_for(100ms);
              serial_running.fetch_sub(1);
              h.succeed(kEmpty);
              serial_done.fetch_add(1);
            }),
        "add_action_server (serial)");
  auto send_serial = client.add_service_client({.service = ract::action_send_goal_service(kSerial),
                                                .type_name = ract::action_send_goal_type(kType)});
  CHECK(send_serial && send_serial->wait_for_server(10s), "serial send_goal client");
  for (uint8_t n = 10; n < 13; ++n) {
    const auto reply = send_serial->call(ract::wrap_send_goal_request(make_id(n), kEmpty), 5s);
    bool accepted = false;
    CHECK(reply && ract::parse_send_goal_response(*reply, accepted) && accepted,
          "serial goal accepted");
  }
  CHECK(wait_until([&] { return serial_done.load() == 3; }, 5s), "the three serial goals ran");
  CHECK(serial_peak.load() == 1, "one at a time on one worker");

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
