// espp action SERVER for the ROS 2 interop matrix: hosts a Fibonacci action
// (example_interfaces/action/Fibonacci) so a ROS 2 client
// (`ros2 action send_goal -f /fibonacci ...` or an rclpy client) can drive it.
// Runs until killed. Prints each goal so the harness can confirm traffic.
//
// Fibonacci CDR (little-endian, post-encap): Goal = {order: int32};
// Result/Feedback = {sequence: int32[]} (uint32 length prefix + elements).

#include <chrono>
#include <cstdint>
#include <cstdio>
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
std::vector<uint8_t> encode_seq(const std::vector<int32_t> &seq) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i32(v, static_cast<int32_t>(seq.size()));
  for (int32_t x : seq)
    put_i32(v, x);
  return v;
}
} // namespace

int main(int argc, char **argv) {
  const char *action = (argc > 1) ? argv[1] : "/fibonacci";
  const char *type = (argc > 2) ? argv[2] : "example_interfaces::action::dds_::Fibonacci";
  const int run_s = (argc > 3) ? std::atoi(argv[3]) : 40;
  const char *interface_ip = (argc > 4) ? argv[4] : "";

  espp::RtpsParticipant p(
      {.interface_address = interface_ip, .log_level = espp::Logger::Verbosity::WARN});
  if (!p.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  std::atomic<int> handled{0};
  if (!p.add_action_server(
          {action, type},
          [](const espp::RtpsParticipant::GoalId &, std::span<const uint8_t> goal) {
            return goal.size() >= 8 && get_i32(goal, 4) > 0;
          },
          [&handled](espp::RtpsParticipant::ActionGoalHandle h) {
            const int32_t order = get_i32(h.goal(), 4);
            std::vector<int32_t> seq{0, 1};
            for (int32_t i = 1; i < order; ++i) {
              seq.push_back(seq[i] + seq[i - 1]);
              auto fb = encode_seq(seq);
              h.publish_feedback({fb.data(), fb.size()});
              std::this_thread::sleep_for(200ms);
            }
            auto result = encode_seq(seq);
            h.succeed({result.data(), result.size()});
            handled.fetch_add(1);
            std::printf("server: goal order=%d done, seq_len=%zu\n", order, seq.size());
            std::fflush(stdout);
          })) {
    std::printf("FAIL: add_action_server\n");
    return 1;
  }

  std::printf("server: ready action=%s type=%s\n", action, type);
  std::fflush(stdout);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(run_s);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(200ms);
  }
  p.stop();
  std::printf("server: handled %d goal(s)\n", handled.load());
  return handled.load() > 0 ? 0 : 2;
}
