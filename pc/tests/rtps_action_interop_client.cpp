// espp action CLIENT for the ROS 2 interop matrix: drives a Fibonacci action
// hosted by a ROS 2 (rclpy) action server, and checks the result sequence.
// Exits 0 iff the terminal result equals Fibonacci(order) with status SUCCEEDED.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
std::vector<uint8_t> encode_goal(int32_t order) {
  std::vector<uint8_t> v(kEncap, kEncap + 4);
  put_i32(v, order);
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

int main(int argc, char **argv) {
  const char *action = (argc > 1) ? argv[1] : "/fibonacci";
  const char *type = (argc > 2) ? argv[2] : "example_interfaces::action::dds_::Fibonacci";
  const int32_t order = (argc > 3) ? std::atoi(argv[3]) : 5;
  const int timeout_s = (argc > 4) ? std::atoi(argv[4]) : 30;
  const char *interface_ip = (argc > 5) ? argv[5] : "";

  espp::RtpsParticipant p(
      {.interface_address = interface_ip, .log_level = espp::Logger::Verbosity::WARN});
  if (!p.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  auto client = p.add_action_client({action, type});
  if (!client) {
    std::printf("FAIL: add_action_client\n");
    return 1;
  }

  std::this_thread::sleep_for(3s); // let the rclpy server + all 5 endpoints match

  std::mutex m;
  std::condition_variable cv;
  bool done = false;
  int feedback_count = 0;
  int8_t status = 0;
  std::vector<int32_t> result;

  auto gid = client->send_goal(
      encode_goal(order),
      [&](std::span<const uint8_t> fb) {
        std::lock_guard<std::mutex> lk(m);
        ++feedback_count;
        (void)decode_seq(fb);
      },
      [&](int8_t st, std::span<const uint8_t> res) {
        std::lock_guard<std::mutex> lk(m);
        status = st;
        result = decode_seq(res);
        done = true;
        cv.notify_one();
      });
  if (!gid.has_value()) {
    std::printf("FAIL: send_goal\n");
    return 1;
  }

  {
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, std::chrono::seconds(timeout_s), [&] { return done; });
  }
  p.stop();

  // Build the expected Fibonacci(order) sequence: [0, 1, 1, 2, ...].
  std::vector<int32_t> expected{0, 1};
  for (int32_t i = 1; i < order; ++i)
    expected.push_back(expected[i] + expected[i - 1]);

  const bool ok = done && status == 4 /*SUCCEEDED*/ && result == expected && feedback_count > 0;
  std::printf("client: status=%d seq_len=%zu feedback=%d => %s\n", (int)status, result.size(),
              feedback_count, ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
