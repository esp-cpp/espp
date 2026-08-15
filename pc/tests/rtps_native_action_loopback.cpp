// In-process native (espp<->espp) action loopback: the lean AMI - one native
// send_goal service + one feedback topic carrying the terminal result. A
// "countup" action: the server counts 1..N, publishing each value as feedback,
// then succeeds with N. Exits 0 iff feedback arrived and the result equals N.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};
void put_i32(std::vector<uint8_t> &v, int32_t x) {
  for (int i = 0; i < 4; ++i)
    v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));
}
int32_t get_i32(std::span<const uint8_t> p, size_t off) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
      (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24));
}
std::vector<uint8_t> encode_i32(int32_t x) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i32(v, x);
  return v;
}
} // namespace

int main() {
  const espp::RtpsParticipant::ActionConfig cfg{"/countup", "espp::native::CountUp"};

  espp::RtpsParticipant server({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  if (!server.add_native_action_server(
          cfg,
          [](std::span<const uint8_t> goal) { return goal.size() >= 8 && get_i32(goal, 4) > 0; },
          [](espp::RtpsParticipant::NativeGoalHandle h) {
            const int32_t n = get_i32(h.goal(), 4);
            for (int32_t i = 1; i <= n; ++i) {
              // Cancel-aware: wind down (CANCELED) when the client cancels.
              if (h.is_canceling()) {
                auto partial = encode_i32(i - 1);
                h.canceled({partial.data(), partial.size()});
                return;
              }
              auto fb = encode_i32(i);
              h.publish_feedback({fb.data(), fb.size()});
              std::this_thread::sleep_for(80ms);
            }
            auto result = encode_i32(n);
            h.succeed({result.data(), result.size()});
          })) {
    std::printf("FAIL: add_native_action_server\n");
    return 1;
  }

  auto action = client.add_native_action_client(cfg);
  if (!action) {
    std::printf("FAIL: add_native_action_client\n");
    return 1;
  }

  std::this_thread::sleep_for(2s); // SEDP match

  std::mutex m;
  std::condition_variable cv;
  bool done = false;
  int feedback_count = 0;
  uint8_t status = 0;
  int32_t result = 0;

  const int32_t n = 5;
  action->send_goal(
      encode_i32(n),
      [&](std::span<const uint8_t> fb) {
        std::lock_guard<std::mutex> lk(m);
        if (fb.size() >= 8)
          ++feedback_count;
      },
      [&](uint8_t st, std::span<const uint8_t> res) {
        std::lock_guard<std::mutex> lk(m);
        status = st;
        if (res.size() >= 8)
          result = get_i32(res, 4);
        done = true;
        cv.notify_one();
      });

  {
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 15s, [&] { return done; });
  }

  const bool ok = done && status == 4 /*SUCCEEDED*/ && result == n && feedback_count > 0;
  std::printf("native action: status=%d result=%d feedback=%d => %s\n", (int)status, result,
              feedback_count, ok ? "PASS" : "FAIL");

  // Phase 2: cancel a long-running goal mid-flight. The client learns the goal
  // handle via on_accepted, waits for the first feedback, then cancel_goal()s it;
  // the server's execute observes is_canceling() and ends CANCELED.
  std::mutex m2;
  std::condition_variable cv2;
  bool done2 = false, have_handle = false;
  int feedback2 = 0;
  uint8_t status2 = 0;
  uint32_t handle2 = 0;
  action->send_goal(
      encode_i32(100), // bounded (100 * 80ms = 8s) so a lost cancel can't block stop()'s join;
                       // the client waits for the first feedback before canceling, so cancel
                       // lands mid-flight (the goal exits within ~80ms of the flag being set)
      [&](std::span<const uint8_t> fb) {
        std::lock_guard<std::mutex> lk(m2);
        if (fb.size() >= 8)
          ++feedback2;
        cv2.notify_one();
      },
      [&](uint8_t st, std::span<const uint8_t>) {
        std::lock_guard<std::mutex> lk(m2);
        status2 = st;
        done2 = true;
        cv2.notify_one();
      },
      [&](uint32_t handle) {
        std::lock_guard<std::mutex> lk(m2);
        handle2 = handle;
        have_handle = true;
        cv2.notify_one();
      });
  {
    std::unique_lock<std::mutex> lk(m2);
    cv2.wait_for(lk, 5s, [&] { return have_handle && feedback2 >= 1; });
  }
  action->cancel_goal(handle2);
  {
    std::unique_lock<std::mutex> lk(m2);
    cv2.wait_for(lk, 10s, [&] { return done2; });
  }
  const bool cancel_ok = done2 && status2 == 5 /*CANCELED*/;
  std::printf("native cancel: handle=%u status=%d feedback=%d => %s\n", handle2, (int)status2,
              feedback2, cancel_ok ? "PASS" : "FAIL");

  server.stop();
  client.stop();

  if (ok && cancel_ok) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
