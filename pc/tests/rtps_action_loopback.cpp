// In-process action (AMI) loopback: one participant hosts a Fibonacci action
// server, another drives it as a client. Exercises the full M2 action path -
// mangling (M2.1), envelope codec (M2.2), the 3 services (send_goal/get_result/
// cancel) + 2 topics (feedback/status), deferred get_result, and goal
// correlation by UUID - without ROS 2. Fibonacci CDR encoding matches
// example_interfaces/action/Fibonacci so the same payloads work against a real
// ROS 2 node in the docker interop leg.
//
// Exits 0 iff the result sequence is Fibonacci(order) and feedback was received.

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
// Fibonacci Goal = { order: int32 }.
std::vector<uint8_t> encode_goal(int32_t order) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i32(v, order);
  return v;
}
// Fibonacci Result/Feedback = { sequence: int32[] } (len-prefixed).
std::vector<uint8_t> encode_seq(const std::vector<int32_t> &seq) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i32(v, static_cast<int32_t>(seq.size()));
  for (int32_t x : seq)
    put_i32(v, x);
  return v;
}
std::vector<int32_t> decode_seq(std::span<const uint8_t> cdr) {
  std::vector<int32_t> seq;
  if (cdr.size() < 8)
    return seq;
  const uint32_t n = static_cast<uint32_t>(get_i32(cdr, 4));
  for (uint32_t i = 0; i < n && 8 + (i + 1) * 4 <= cdr.size(); ++i)
    seq.push_back(get_i32(cdr, 8 + i * 4));
  return seq;
}
} // namespace

int main() {
  const espp::RtpsParticipant::ActionConfig cfg{"/fibonacci",
                                                "example_interfaces::action::dds_::Fibonacci"};

  espp::RtpsParticipant server({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant client({.log_level = espp::Logger::Verbosity::WARN});
  if (!server.start() || !client.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  // Server: compute the Fibonacci sequence, publishing feedback each step.
  if (!server.add_action_server(
          cfg,
          [](const espp::RtpsParticipant::GoalId &, std::span<const uint8_t> goal) {
            return goal.size() >= 8 && get_i32(goal, 4) > 0; // accept order > 0
          },
          [](espp::RtpsParticipant::ActionGoalHandle h) {
            const int32_t order = get_i32(h.goal(), 4);
            std::vector<int32_t> seq{0, 1};
            for (int32_t i = 1; i < order; ++i) {
              seq.push_back(seq[i] + seq[i - 1]);
              auto fb = encode_seq(seq);
              h.publish_feedback({fb.data(), fb.size()});
              std::this_thread::sleep_for(100ms);
            }
            auto result = encode_seq(seq);
            h.succeed({result.data(), result.size()});
          })) {
    std::printf("FAIL: add_action_server\n");
    return 1;
  }

  auto action = client.add_action_client(cfg);
  if (!action) {
    std::printf("FAIL: add_action_client\n");
    return 1;
  }

  std::this_thread::sleep_for(2s); // SEDP match across all 5 endpoints

  std::mutex m;
  std::condition_variable cv;
  bool done = false;
  int feedback_count = 0;
  std::vector<int32_t> result_seq;
  int8_t result_status = 0;

  const int32_t order = 5;
  auto gid = action->send_goal(
      encode_goal(order),
      [&](std::span<const uint8_t> fb) {
        std::lock_guard<std::mutex> lk(m);
        ++feedback_count;
        (void)decode_seq(fb);
      },
      [&](int8_t status, std::span<const uint8_t> result) {
        std::lock_guard<std::mutex> lk(m);
        result_status = status;
        result_seq = decode_seq(result);
        done = true;
        cv.notify_one();
      });
  if (!gid.has_value()) {
    std::printf("FAIL: send_goal\n");
    return 1;
  }

  {
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 20s, [&] { return done; });
  }

  server.stop();
  client.stop();

  // Fibonacci(5) = [0, 1, 1, 2, 3, 5]; feedback should have arrived (order-1 msgs).
  const std::vector<int32_t> expected{0, 1, 1, 2, 3, 5};
  const bool result_ok = (result_status == 4 /*SUCCEEDED*/) && (result_seq == expected);
  const bool feedback_ok = (feedback_count > 0);
  std::printf("result status=%d seq_len=%zu feedback=%d => %s\n", (int)result_status,
              result_seq.size(), feedback_count, (result_ok && feedback_ok) ? "PASS" : "FAIL");
  if (result_ok && feedback_ok) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
