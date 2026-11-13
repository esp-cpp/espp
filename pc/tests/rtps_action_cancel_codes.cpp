// A cancel request is answered with the ROS 2 verdict (action_msgs/srv/
// CancelGoal): the goals now canceling, and a return code saying why none is.
//
// The server used to reply a constant {return_code: 0, goals_canceling: []}
// whatever happened. rclpy's canonical client judges a cancel by
// len(goals_canceling) > 0, so every rclpy user saw "Goal failed to cancel"
// although the goal did cancel; rclcpp clients judging by the code saw
// success when cancelling a goal the server never had. Now:
//  - a live goal the server agrees to cancel: code 0 (ERROR_NONE), the goal
//    listed, CANCELING published on the status topic, is_canceling() true;
//  - on_cancel refuses: 1 (ERROR_REJECTED), nothing listed;
//  - an unknown goal id: 2 (ERROR_UNKNOWN_GOAL_ID);
//  - a goal already finished (still retained for get_result): 3
//    (ERROR_GOAL_TERMINATED);
//  - a zero goal id cancels every active goal and lists them all;
//  - the stamp rules of rcl_action: a zero id with a stamp selects the active
//    goals accepted at or before the stamp (none for a stamp in the past); an
//    id with a stamp selects that goal and the active goals accepted at or
//    before the stamp. Goals are stamped on the wall clock at acceptance,
//    as the SendGoal_Response reports.
//
// Byte-level API, one participant hosting, one calling. Exits 0 on success,
// 1 on failure.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "rtps/rpc/action_naming.hpp"
#include "rtps/rpc/action_types.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;
using Rtps = espp::RtpsParticipant;

namespace {
#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

// Goal = { mode: int32 }: 0 finishes at once, anything else runs until cancelled.
std::vector<uint8_t> encode_goal(int32_t mode) {
  std::vector<uint8_t> v{0x00, 0x01, 0x00, 0x00};
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((static_cast<uint32_t>(mode) >> (8 * i)) & 0xFF));
  }
  return v;
}
int32_t goal_mode(std::span<const uint8_t> goal) {
  return goal.size() >= 8
             ? static_cast<int32_t>(goal[4] | (goal[5] << 8) | (goal[6] << 16) | (goal[7] << 24))
             : 0;
}
// Result = {} (an empty message: one placeholder byte, as ROS 2 encodes it).
const std::vector<uint8_t> kEmptyResult{0x00, 0x01, 0x00, 0x00, 0x00};

struct Verdict {
  std::mutex m;
  std::condition_variable cv;
  bool done{false};
  int8_t code{-1};
  std::size_t canceling{0};
};

// Send a cancel for `id` and wait for the server's verdict.
std::optional<std::pair<int8_t, std::size_t>> cancel(Rtps::ActionClient &client,
                                                     const Rtps::GoalId &id) {
  auto v = std::make_shared<Verdict>();
  if (!client.cancel_goal(id, [v](int8_t code, std::size_t canceling) {
        std::lock_guard<std::mutex> lock(v->m);
        v->code = code;
        v->canceling = canceling;
        v->done = true;
        v->cv.notify_all();
      })) {
    return std::nullopt;
  }
  std::unique_lock<std::mutex> lock(v->m);
  if (!v->cv.wait_for(lock, 10s, [&] { return v->done; })) {
    return std::nullopt;
  }
  return std::make_pair(v->code, v->canceling);
}

struct GoalOutcome {
  std::mutex m;
  std::condition_variable cv;
  bool done{false};
  int8_t status{0};
};

bool run() {
  Rtps server({.log_level = espp::Logger::Verbosity::WARN});
  Rtps client({.log_level = espp::Logger::Verbosity::WARN});
  CHECK(server.start() && client.start(), "start");

  std::atomic<bool> allow_cancel{true};
  std::atomic<int> active{0};
  const Rtps::ActionConfig cfg{.action = "/cancel_codes",
                               .type_name = "espp_test::action::dds_::CancelCodes"};
  CHECK(server.add_action_server(
            cfg, [](const Rtps::GoalId &, std::span<const uint8_t>) { return true; },
            [&active](Rtps::ActionGoalHandle h) {
              active.fetch_add(1);
              if (goal_mode(h.goal()) != 0) {
                while (!h.is_canceling()) {
                  std::this_thread::sleep_for(5ms);
                }
                active.fetch_sub(1);
                h.canceled(kEmptyResult);
                return;
              }
              active.fetch_sub(1);
              h.succeed(kEmptyResult);
            },
            [&allow_cancel](const Rtps::GoalId &) { return allow_cancel.load(); }),
        "add_action_server");
  auto act = client.add_action_client(cfg);
  CHECK(act != nullptr, "add_action_client");
  CHECK(act->wait_for_server(10s), "wait_for_server");
  std::this_thread::sleep_for(300ms); // feedback/status topics match alongside

  const auto send =
      [&](int32_t mode) -> std::pair<std::optional<Rtps::GoalId>, std::shared_ptr<GoalOutcome>> {
    auto out = std::make_shared<GoalOutcome>();
    auto id =
        act->send_goal(encode_goal(mode), nullptr, [out](int8_t status, std::span<const uint8_t>) {
          std::lock_guard<std::mutex> lock(out->m);
          out->status = status;
          out->done = true;
          out->cv.notify_all();
        });
    return {id, out};
  };
  const auto wait_done = [](const std::shared_ptr<GoalOutcome> &out) {
    std::unique_lock<std::mutex> lock(out->m);
    return out->cv.wait_for(lock, 10s, [&] { return out->done; });
  };
  const auto wait_active = [&](int n) {
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (active.load() < n && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(5ms);
    }
    return active.load() >= n;
  };

  // Unknown goal id: 2.
  Rtps::GoalId unknown{};
  unknown.fill(0x5a);
  auto verdict = cancel(*act, unknown);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_UNKNOWN_GOAL_ID &&
            verdict->second == 0,
        "cancel of an unknown goal: ERROR_UNKNOWN_GOAL_ID, nothing canceling");

  // A live goal the server refuses to cancel: 1.
  auto [id1, out1] = send(1);
  CHECK(id1.has_value() && wait_active(1), "goal 1 running");
  allow_cancel = false;
  verdict = cancel(*act, *id1);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_REJECTED && verdict->second == 0,
        "cancel refused by on_cancel: ERROR_REJECTED, nothing canceling");
  // ... then allowed: 0, the goal listed, and it ends CANCELED.
  allow_cancel = true;
  verdict = cancel(*act, *id1);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_NONE && verdict->second == 1,
        "cancel of a live goal: ERROR_NONE, one goal canceling");
  CHECK(wait_done(out1) && out1->status == static_cast<int8_t>(rtps::rpc::GoalStatus::CANCELED),
        "the goal ends CANCELED");

  // A finished goal (its result fetched, still retained): 3.
  auto [id0, out0] = send(0);
  CHECK(id0.has_value() && wait_done(out0) &&
            out0->status == static_cast<int8_t>(rtps::rpc::GoalStatus::SUCCEEDED),
        "goal 0 finished");
  verdict = cancel(*act, *id0);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_GOAL_TERMINATED &&
            verdict->second == 0,
        "cancel of a finished goal: ERROR_GOAL_TERMINATED");

  // Two live goals, a zero id: both listed, both end CANCELED.
  auto [id2, out2] = send(2);
  auto [id3, out3] = send(3);
  CHECK(id2.has_value() && id3.has_value() && wait_active(2), "goals 2 and 3 running");
  Rtps::GoalId all{};
  verdict = cancel(*act, all);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_NONE && verdict->second == 2,
        "cancel all: ERROR_NONE, two goals canceling");
  CHECK(wait_done(out2) && wait_done(out3) &&
            out2->status == static_cast<int8_t>(rtps::rpc::GoalStatus::CANCELED) &&
            out3->status == static_cast<int8_t>(rtps::rpc::GoalStatus::CANCELED),
        "both end CANCELED");

  // The stamp rules, through a service client on the cancel service (the
  // action client's cancel_goal() sends no stamp) from a third participant -
  // the action client already holds that service's client in `client`.
  Rtps stamper({.log_level = espp::Logger::Verbosity::WARN});
  CHECK(stamper.start(), "start (stamper)");
  auto cancel_svc =
      stamper.add_service_client({.service = rtps::rpc::action_cancel_goal_service(cfg.action),
                                  .type_name = rtps::rpc::action_cancel_goal_type()});
  CHECK(cancel_svc && cancel_svc->wait_for_server(10s), "cancel service client");
  const auto stamped_cancel = [&](const Rtps::GoalId &id,
                                  std::chrono::system_clock::time_point stamp)
      -> std::optional<std::pair<int8_t, std::size_t>> {
    const auto since_epoch = stamp.time_since_epoch();
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - sec);
    const auto reply =
        cancel_svc->call(rtps::rpc::make_cancel_goal_request(id, static_cast<int32_t>(sec.count()),
                                                             static_cast<uint32_t>(nsec.count())),
                         5s);
    int8_t code = -1;
    std::vector<rtps::rpc::GoalUuid> canceling;
    if (!reply || !rtps::rpc::parse_cancel_goal_response(*reply, code, canceling)) {
      return std::nullopt;
    }
    return std::make_pair(code, canceling.size());
  };
  // Goals A, B, C accepted in order, with gaps the wall clock can tell apart.
  auto [idA, outA] = send(4);
  CHECK(idA.has_value() && wait_active(1), "goal A running");
  std::this_thread::sleep_for(60ms);
  const auto between_a_b = std::chrono::system_clock::now();
  std::this_thread::sleep_for(60ms);
  auto [idB, outB] = send(5);
  CHECK(idB.has_value() && wait_active(2), "goal B running");
  std::this_thread::sleep_for(60ms);
  auto [idC, outC] = send(6);
  CHECK(idC.has_value() && wait_active(3), "goal C running");
  std::this_thread::sleep_for(60ms);
  const auto after_c = std::chrono::system_clock::now();

  // A stamp in the past selects nothing: ERROR_NONE, none canceling.
  verdict = stamped_cancel(all, between_a_b - std::chrono::hours(1));
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_NONE && verdict->second == 0,
        "zero id + a stamp before every goal: ERROR_NONE, nothing canceling");
  // A zero id with a stamp between A and B: only A.
  verdict = stamped_cancel(all, between_a_b);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_NONE && verdict->second == 1,
        "zero id + stamp between A and B: only A canceling");
  CHECK(wait_done(outA) && outA->status == static_cast<int8_t>(rtps::rpc::GoalStatus::CANCELED),
        "A ends CANCELED");
  // C's id with a stamp after everything: C, and B (accepted before the
  // stamp, still active; A is finished and not listed).
  verdict = stamped_cancel(*idC, after_c);
  CHECK(verdict && verdict->first == rtps::rpc::CANCEL_ERROR_NONE && verdict->second == 2,
        "C's id + a stamp after all: C and B canceling");
  CHECK(wait_done(outB) && wait_done(outC) &&
            outB->status == static_cast<int8_t>(rtps::rpc::GoalStatus::CANCELED) &&
            outC->status == static_cast<int8_t>(rtps::rpc::GoalStatus::CANCELED),
        "B and C end CANCELED");

  stamper.stop();
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
