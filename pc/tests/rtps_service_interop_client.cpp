// espp service CLIENT for the ROS 2 interop matrix: calls an add_two_ints
// service hosted by a ROS 2 (rclpy) server and checks the returned sum. Exits 0
// iff the correlated reply equals a + b - and, with a pipeline depth above one,
// iff every one of that many simultaneously outstanding requests gets its own
// correct reply (replies may come back in any order; each must land on the
// request it answers).
//
// usage: rtps_service_interop_client [service] [type] [a] [b] [timeout_s]
//                                     [interface_ip] [pipeline]
//
// AddTwoInts wire form (classic CDR, little-endian): request = int64 a + int64 b;
// response = int64 sum. Payloads include the 4-byte CDR encapsulation header.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
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
std::vector<uint8_t> encode_request(int64_t a, int64_t b) {
  std::vector<uint8_t> request(kEncap, kEncap + 4);
  put_i64(request, a);
  put_i64(request, b);
  return request;
}
} // namespace

int main(int argc, char **argv) {
  const char *service = (argc > 1) ? argv[1] : "/add_two_ints";
  const char *type = (argc > 2) ? argv[2] : "example_interfaces::srv::dds_::AddTwoInts";
  const int64_t a = (argc > 3) ? std::atoll(argv[3]) : 20;
  const int64_t b = (argc > 4) ? std::atoll(argv[4]) : 22;
  const int timeout_s = (argc > 5) ? std::atoi(argv[5]) : 30;
  const char *interface_ip = (argc > 6) ? argv[6] : "";
  const int pipeline = (argc > 7) ? std::max(1, std::atoi(argv[7])) : 1;

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

  const auto request = encode_request(a, b);

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
  std::printf("client: %lld + %lld = %lld (expected %lld) => %s\n", (long long)a, (long long)b,
              (long long)sum, (long long)(a + b), ok ? "PASS" : "FAIL");

  // Pipelined: `pipeline` requests in flight at once, each with its own
  // operands, so a reply delivered to the wrong pending request shows up as a
  // wrong sum rather than passing by coincidence.
  if (ok && pipeline > 1) {
    int correct = 0;
    std::vector<std::future<std::optional<std::vector<uint8_t>>>> futures;
    futures.reserve(pipeline);
    for (int i = 0; i < pipeline; ++i) {
      futures.push_back(client->call_future(encode_request(a + i, b + 2 * i)));
    }
    for (int i = 0; i < pipeline; ++i) {
      if (futures[i].wait_until(deadline) != std::future_status::ready) {
        std::printf("client: pipelined request %d: no reply\n", i);
        continue;
      }
      auto reply = futures[i].get();
      if (!reply.has_value() || reply->size() < 4 + 8) {
        std::printf("client: pipelined request %d: failed\n", i);
        continue;
      }
      const int64_t got = get_i64(*reply, 4);
      const int64_t expected = (a + i) + (b + 2 * i);
      if (got == expected) {
        ++correct;
      } else {
        std::printf("client: pipelined request %d: got %lld, expected %lld\n", i, (long long)got,
                    (long long)expected);
      }
    }
    ok = (correct == pipeline);
    std::printf("client: pipelined %d/%d correct => %s\n", correct, pipeline, ok ? "PASS" : "FAIL");
  }

  p.stop();
  return ok ? 0 : 1;
}
