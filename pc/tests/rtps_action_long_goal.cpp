// A goal that outlives the service request timeout still delivers its result.
//
// An action client's get_result request waits for the whole goal. The
// service client it rides on prunes requests older than
// ServiceConfig::request_timeout (30 s by default), so a goal running longer
// than that was abandoned at the client's next request and reported ABORTED
// while the server was still executing it. The get_result client now has no
// request timeout (ActionConfig::request_timeout applies to send_goal and
// cancel_goal only): the result arrives when the goal finishes.
//
// Byte-level client with request_timeout 500 ms against a goal that runs 2 s;
// a second goal is sent 1 s in (what used to trigger the prune). The first
// goal must report SUCCEEDED, after about 2 s.
//
// Exits 0 on success, 1 on failure.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "rtps/rpc/action_types.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;
using Rtps = espp::RtpsParticipant;

namespace {
// Goal = { ms: int32 } to sleep; Result = {} (one placeholder byte).
std::vector<uint8_t> encode_goal(int32_t ms) {
  std::vector<uint8_t> v{0x00, 0x01, 0x00, 0x00};
  for (int i = 0; i < 4; ++i) {
    v.push_back(static_cast<uint8_t>((static_cast<uint32_t>(ms) >> (8 * i)) & 0xFF));
  }
  return v;
}
int32_t goal_ms(std::span<const uint8_t> goal) {
  return goal.size() >= 8
             ? static_cast<int32_t>(goal[4] | (goal[5] << 8) | (goal[6] << 16) | (goal[7] << 24))
             : 0;
}
const std::vector<uint8_t> kEmptyResult{0x00, 0x01, 0x00, 0x00, 0x00};

struct Outcome {
  std::mutex m;
  std::condition_variable cv;
  bool done{false};
  int8_t status{0};
  std::chrono::steady_clock::time_point at{};
};
} // namespace

int main() {
  Rtps server({.log_level = espp::Logger::Verbosity::WARN});
  Rtps client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  const Rtps::ActionConfig server_cfg{.action = "/long_goal",
                                      .type_name = "espp_test::action::dds_::LongGoal"};
  if (!server.add_action_server(
          server_cfg, [](const Rtps::GoalId &, std::span<const uint8_t>) { return true; },
          [](Rtps::ActionGoalHandle h) {
            const auto deadline =
                std::chrono::steady_clock::now() + std::chrono::milliseconds(goal_ms(h.goal()));
            while (std::chrono::steady_clock::now() < deadline && !h.is_canceling()) {
              std::this_thread::sleep_for(10ms);
            }
            h.succeed(kEmptyResult);
          })) {
    std::printf("FAIL: add_action_server\n");
    return 1;
  }
  auto act = client.add_action_client({.action = "/long_goal",
                                       .type_name = "espp_test::action::dds_::LongGoal",
                                       .request_timeout = 500ms});
  if (!act || !act->wait_for_server(10s)) {
    std::printf("FAIL: add_action_client / wait_for_server\n");
    return 1;
  }
  std::this_thread::sleep_for(300ms); // feedback/status topics match alongside

  const auto send = [&](int32_t ms) {
    auto out = std::make_shared<Outcome>();
    auto id =
        act->send_goal(encode_goal(ms), nullptr, [out](int8_t status, std::span<const uint8_t>) {
          std::lock_guard<std::mutex> lock(out->m);
          out->status = status;
          out->at = std::chrono::steady_clock::now();
          out->done = true;
          out->cv.notify_all();
        });
    return std::make_pair(id, out);
  };
  const auto t0 = std::chrono::steady_clock::now();
  auto [id_long, out_long] = send(2000);
  if (!id_long) {
    std::printf("FAIL: send_goal (long)\n");
    return 1;
  }
  std::this_thread::sleep_for(1s);
  auto [id_short, out_short] = send(10); // the next request: used to prune the first
  if (!id_short) {
    std::printf("FAIL: send_goal (short)\n");
    return 1;
  }
  {
    std::unique_lock<std::mutex> lock(out_long->m);
    if (!out_long->cv.wait_for(lock, 10s, [&] { return out_long->done; })) {
      std::printf("FAIL: no result for the long goal\n");
      return 1;
    }
  }
  const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(out_long->at - t0);
  std::printf("long goal: status %d after %lld ms\n", static_cast<int>(out_long->status),
              static_cast<long long>(took.count()));
  if (out_long->status != static_cast<int8_t>(rtps::rpc::GoalStatus::SUCCEEDED)) {
    std::printf("FAIL: the long goal was reported %d (ABORTED = 6) while it was still running\n",
                static_cast<int>(out_long->status));
    return 1;
  }
  if (took < 1900ms) {
    std::printf("FAIL: the result came before the goal could have finished\n");
    return 1;
  }
  {
    std::unique_lock<std::mutex> lock(out_short->m);
    if (!out_short->cv.wait_for(lock, 10s, [&] { return out_short->done; }) ||
        out_short->status != static_cast<int8_t>(rtps::rpc::GoalStatus::SUCCEEDED)) {
      std::printf("FAIL: the short goal did not succeed\n");
      return 1;
    }
  }
  client.stop();
  server.stop();
  std::printf("PASS\n");
  return 0;
}
