// espp service SERVER for the ROS 2 interop matrix hosting a std_srvs/srv/Trigger
// service: the request body is EMPTY (ROS 2 serializes it as one padding byte),
// the reply is a bool plus a string whose length the harness chooses - so one
// server covers the smallest request a ROS 2 client can send and a reply near
// the single-DATA budget. Runs until killed; prints each handled request.
//
// usage: rtps_trigger_interop_server [service] [type] [run_s] [message_len]
//                                    [interface_ip]
//
// Trigger wire form (classic CDR, little-endian): request = uint8 (the
// structure_needs_at_least_one_member placeholder, ignored here); response =
// bool success (1 byte, padded to 4) + string message (uint32 length including
// the NUL, the characters, NUL). Payloads include the 4-byte CDR encapsulation.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};
std::vector<uint8_t> encode_response(bool success, const std::string &message) {
  std::vector<uint8_t> b(kEncap, kEncap + 4);
  b.push_back(success ? 1 : 0);
  b.insert(b.end(), 3, 0); // pad to the string's 4-byte alignment
  const uint32_t len = static_cast<uint32_t>(message.size() + 1);
  for (int i = 0; i < 4; ++i) {
    b.push_back(static_cast<uint8_t>((len >> (8 * i)) & 0xFF));
  }
  b.insert(b.end(), message.begin(), message.end());
  b.push_back(0);
  return b;
}
} // namespace

int main(int argc, char **argv) {
  const char *service = (argc > 1) ? argv[1] : "/trigger";
  const char *type = (argc > 2) ? argv[2] : "std_srvs::srv::dds_::Trigger";
  const int run_s = (argc > 3) ? std::atoi(argv[3]) : 30;
  const int message_len = (argc > 4) ? std::atoi(argv[4]) : 16;
  const char *interface_ip = (argc > 5) ? argv[5] : "";

  espp::RtpsParticipant p(
      {.interface_address = interface_ip, .log_level = espp::Logger::Verbosity::WARN});
  if (!p.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  const std::string message(static_cast<size_t>(std::max(0, message_len)), 'x');
  const auto response = encode_response(true, message);
  std::atomic<int> handled{0};
  if (!p.add_service_server({service, type},
                            [&](std::span<const uint8_t> req) -> std::vector<uint8_t> {
                              handled.fetch_add(1);
                              std::printf("server: trigger request (%zu bytes) -> success, "
                                          "message_len=%zu\n",
                                          req.size(), message.size());
                              std::fflush(stdout);
                              return response;
                            })) {
    std::printf("FAIL: add_service_server\n");
    return 1;
  }

  std::printf("server: ready service=%s type=%s reply=%zu bytes\n", service, type, response.size());
  std::fflush(stdout);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(run_s);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(200ms);
  }
  p.stop();
  std::printf("server: handled %d request(s)\n", handled.load());
  return handled.load() > 0 ? 0 : 2;
}
