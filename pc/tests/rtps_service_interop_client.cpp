// espp service CLIENT for the ROS 2 interop matrix: calls an add_two_ints
// service hosted by a ROS 2 (rclpy) server and checks the returned sum. Exits 0
// iff the correlated reply equals a + b.
//
// AddTwoInts wire form (classic CDR, little-endian): request = int64 a + int64 b;
// response = int64 sum. Payloads include the 4-byte CDR encapsulation header.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};
void put_i64(std::vector<uint8_t> &v, int64_t x) {
  for (int i = 0; i < 8; ++i) {
    v.push_back(static_cast<uint8_t>((x >> (8 * i)) & 0xFF));
  }
}
int64_t get_i64(std::span<const uint8_t> p, size_t off) {
  int64_t x = 0;
  for (int i = 0; i < 8; ++i) {
    x |= static_cast<int64_t>(p[off + i]) << (8 * i);
  }
  return x;
}
} // namespace

int main(int argc, char **argv) {
  const char *service = (argc > 1) ? argv[1] : "/add_two_ints";
  const char *type = (argc > 2) ? argv[2] : "example_interfaces::srv::dds_::AddTwoInts";
  const int64_t a = (argc > 3) ? std::atoll(argv[3]) : 20;
  const int64_t b = (argc > 4) ? std::atoll(argv[4]) : 22;
  const int timeout_s = (argc > 5) ? std::atoi(argv[5]) : 30;
  const char *interface_ip = (argc > 6) ? argv[6] : "";

  espp::RtpsParticipant p(
      {.interface_address = interface_ip, .log_level = espp::Logger::Verbosity::WARN});
  if (!p.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  auto client = p.add_service_client({service, type});
  if (!client) {
    std::printf("FAIL: add_service_client\n");
    return 1;
  }

  std::vector<uint8_t> request(kEncap, kEncap + 4);
  put_i64(request, a);
  put_i64(request, b);

  // Retry: the rclpy server may still be coming up / matching over SEDP.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
  int64_t sum = 0;
  bool ok = false;
  while (!ok && std::chrono::steady_clock::now() < deadline) {
    auto reply = client->call(request, 3s);
    if (reply.has_value() && reply->size() >= 4 + 8) {
      sum = get_i64(*reply, 4);
      ok = (sum == a + b);
    }
    if (!ok) {
      std::this_thread::sleep_for(500ms);
    }
  }

  p.stop();
  std::printf("client: %lld + %lld = %lld (expected %lld) => %s\n", (long long)a, (long long)b,
              (long long)sum, (long long)(a + b), ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
