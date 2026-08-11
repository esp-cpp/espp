// embeddedRTPS interop publisher (Phase 0c of components/rtps_embedded/REFACTOR_PLAN.md).
//
// Publishes CDR string samples on a topic so an external DDS peer (FastDDS or a
// ROS 2 node via rmw_fastrtps) can subscribe. Topic/type default to the ROS 2
// conventions for std_msgs/String on /chatter.
//
// Usage: rtps_embedded_interop_pub [topic] [type] [reliable(0|1)] [count] [period_ms]
// Exits 0 after publishing `count` samples.

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

int main(int argc, char **argv) {
  const char *topic = (argc > 1) ? argv[1] : "rt/chatter";
  const char *type = (argc > 2) ? argv[2] : "std_msgs::msg::dds_::String_";
  const bool reliable = (argc > 3) ? (std::atoi(argv[3]) != 0) : true;
  const int count = (argc > 4) ? std::atoi(argv[4]) : 30;
  const int period_ms = (argc > 5) ? std::atoi(argv[5]) : 200;

  const std::string ip_str = rtps_test::guess_local_ipv4();
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (std::sscanf(ip_str.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) {
    std::printf("FAIL: could not parse local ip '%s'\n", ip_str.c_str());
    return 1;
  }
  const rtps::Ip4AddressBytes ip{static_cast<uint8_t>(a), static_cast<uint8_t>(b),
                                 static_cast<uint8_t>(c), static_cast<uint8_t>(d)};
  std::printf("interop_pub: ip=%s topic=%s type=%s reliable=%d count=%d\n", ip_str.c_str(), topic,
              type, reliable ? 1 : 0, count);

  static rtps::Domain domain(ip);
  rtps::Participant *part = domain.createParticipant();
  if (part == nullptr || !domain.completeInit()) {
    std::printf("FAIL: participant/init\n");
    return 1;
  }
  rtps::Writer *writer = domain.createWriter(*part, topic, type, reliable);
  if (writer == nullptr) {
    std::printf("FAIL: createWriter\n");
    domain.stop();
    return 1;
  }

  // Give SPDP/SEDP a moment to match before the first sample.
  std::this_thread::sleep_for(2s);

  int sent = 0;
  for (int i = 0; i < count; i++) {
    char text[96];
    std::snprintf(text, sizeof(text), "espp interop %d", i);
    uint8_t cdr_buf[4 + 4 + 96];
    cdr_buf[0] = 0x00; // CDR_LE encapsulation
    cdr_buf[1] = 0x01;
    cdr_buf[2] = 0x00;
    cdr_buf[3] = 0x00;
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset_endian(&ub, cdr_buf, sizeof(cdr_buf), 0, 4,
                                          UCDR_LITTLE_ENDIANNESS);
    if (ucdr_serialize_string(&ub, text) && !ucdr_buffer_has_error(&ub)) {
      const auto total = static_cast<rtps::DataSize_t>(4 + ucdr_buffer_length(&ub));
      if (writer->newChange(rtps::ChangeKind_t::ALIVE, cdr_buf, total) != nullptr) {
        sent++;
        std::printf("sent %d\n", sent);
        std::fflush(stdout);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  // Let reliable retransmits drain before tearing down.
  std::this_thread::sleep_for(1s);
  std::printf("DONE sent=%d\n", sent);
  domain.stop();
  return sent == count ? 0 : 1;
}
