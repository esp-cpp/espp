// embeddedRTPS interop subscriber (Phase 0c of components/rtps_embedded/REFACTOR_PLAN.md).
//
// Subscribes to CDR string samples published by an external DDS peer (FastDDS or a
// ROS 2 node via rmw_fastrtps). Topic/type default to the ROS 2 conventions for
// std_msgs/String on /chatter.
//
// Usage: rtps_embedded_interop_sub [topic] [type] [reliable(0|1)] [required] [timeout_s]
// Exits 0 once `required` samples arrive within `timeout_s`.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "rtps/entities/Domain.hpp"
#include "ucdr/microcdr.h"

#include "rtps_common.hpp"

using namespace std::chrono_literals;

namespace {
std::atomic<int> g_received{0};

void reader_cb(void * /*callee*/, const rtps::ReaderCacheChange &change) {
  const rtps::DataSize_t size = change.getDataSize();
  if (size < 8) { // 4B encapsulation + 4B string length at minimum
    return;
  }
  uint8_t raw[4 + 4 + 256];
  const auto copy_size = static_cast<rtps::DataSize_t>(size <= sizeof(raw) ? size : sizeof(raw));
  if (!change.copyInto(raw, copy_size)) {
    return;
  }
  // Deserialize the CDR string (skip the 4-byte encapsulation header).
  ucdrBuffer ub;
  ucdr_init_buffer_origin_offset_endian(&ub, raw + 4, copy_size - 4, 0, 0,
                                        (raw[1] & 0x01) ? UCDR_LITTLE_ENDIANNESS
                                                        : UCDR_BIG_ENDIANNESS);
  char text[256] = {0};
  if (ucdr_deserialize_string(&ub, text, sizeof(text)) && !ucdr_buffer_has_error(&ub)) {
    const int n = g_received.fetch_add(1) + 1;
    std::printf("received %d: '%s'\n", n, text);
    std::fflush(stdout);
  }
}
} // namespace

int main(int argc, char **argv) {
  const char *topic = (argc > 1) ? argv[1] : "rt/chatter";
  const char *type = (argc > 2) ? argv[2] : "std_msgs::msg::dds_::String_";
  const bool reliable = (argc > 3) ? (std::atoi(argv[3]) != 0) : true;
  const int required = (argc > 4) ? std::atoi(argv[4]) : 5;
  const int timeout_s = (argc > 5) ? std::atoi(argv[5]) : 30;

  const std::string ip_str = rtps_test::guess_local_ipv4();
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(ip_str.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
    std::printf("FAIL: could not parse local ip '%s'\n", ip_str.c_str());
    return 1;
  }
  const rtps::Ip4AddressBytes ip{static_cast<uint8_t>(a), static_cast<uint8_t>(b),
                                 static_cast<uint8_t>(c), static_cast<uint8_t>(d)};
  std::printf("interop_sub: ip=%s topic=%s type=%s reliable=%d required=%d timeout=%ds\n",
              ip_str.c_str(), topic, type, reliable ? 1 : 0, required, timeout_s);

  static rtps::Domain domain(ip);
  rtps::Participant *part = domain.createParticipant();
  if (part == nullptr || !domain.completeInit()) {
    std::printf("FAIL: participant/init\n");
    return 1;
  }
  rtps::Reader *reader = domain.createReader(*part, topic, type, reliable);
  if (reader == nullptr) {
    std::printf("FAIL: createReader\n");
    domain.stop();
    return 1;
  }
  if (reader->registerCallback(reader_cb, nullptr) == 0) {
    std::printf("FAIL: registerCallback\n");
    domain.stop();
    return 1;
  }

  const auto start = std::chrono::steady_clock::now();
  while (g_received.load() < required &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout_s)) {
    std::this_thread::sleep_for(100ms);
  }

  const int received = g_received.load();
  std::printf("%s received=%d required=%d\n", received >= required ? "PASS" : "FAIL", received,
              required);
  domain.stop();
  return received >= required ? 0 : 1;
}
