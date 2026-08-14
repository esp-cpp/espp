// espp service SERVER for the ROS 2 interop matrix: hosts an add_two_ints
// service (example_interfaces/srv/AddTwoInts) so a ROS 2 client
// (`ros2 service call /add_two_ints ...` or an rclpy client) can call it. Runs
// until killed. Prints each handled request so the harness can confirm traffic.
//
// AddTwoInts wire form (classic CDR, little-endian): request = int64 a + int64 b;
// response = int64 sum. Payloads include the 4-byte CDR encapsulation header.

#include <atomic>
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
int64_t get_i64(std::span<const uint8_t> p, size_t off) {
  int64_t x = 0;
  for (int i = 0; i < 8; ++i) {
    x |= static_cast<int64_t>(p[off + i]) << (8 * i);
  }
  return x;
}
std::vector<uint8_t> encode_i64(int64_t v) {
  std::vector<uint8_t> b(kEncap, kEncap + 4);
  for (int i = 0; i < 8; ++i) {
    b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
  return b;
}
} // namespace

int main(int argc, char **argv) {
  const char *service = (argc > 1) ? argv[1] : "/add_two_ints";
  const char *type = (argc > 2) ? argv[2] : "example_interfaces::srv::dds_::AddTwoInts";
  const int run_s = (argc > 3) ? std::atoi(argv[3]) : 30;
  const char *interface_ip = (argc > 4) ? argv[4] : "";

  espp::RtpsParticipant p(
      {.interface_address = interface_ip, .log_level = espp::Logger::Verbosity::WARN});
  if (!p.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  std::atomic<int> handled{0};
  if (!p.add_service_server(
          {service, type}, [&](std::span<const uint8_t> req) -> std::vector<uint8_t> {
            if (req.size() < 4 + 16) {
              return {};
            }
            const int64_t a = get_i64(req, 4);
            const int64_t b = get_i64(req, 12);
            const int64_t sum = a + b;
            handled.fetch_add(1);
            std::printf("server: %lld + %lld = %lld\n", (long long)a, (long long)b, (long long)sum);
            std::fflush(stdout);
            return encode_i64(sum);
          })) {
    std::printf("FAIL: add_service_server\n");
    return 1;
  }

  std::printf("server: ready service=%s type=%s\n", service, type);
  std::fflush(stdout);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(run_s);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(200ms);
  }
  p.stop();
  std::printf("server: handled %d request(s)\n", handled.load());
  return handled.load() > 0 ? 0 : 2;
}
