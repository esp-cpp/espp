// RTPS interop publisher (Phase 1 of components/rtps_embedded/REFACTOR_PLAN.md).
//
// Exercises the espp::RtpsParticipant facade end-to-end: publishes CDR string
// samples (serialized with the reflection-driven cdr::serialize) so an external DDS peer (FastDDS
// or a ROS 2 node via rmw_fastrtps) can subscribe. Defaults follow the ROS 2 conventions for
// std_msgs/String on /chatter.
//
// Usage: rtps_embedded_interop_pub [topic] [type] [reliable(0|1)] [count] [period_ms]
// [interface_ip] [payload_bytes]
// When payload_bytes > 0, publishes a String whose data is a deterministic
// payload_bytes-long ASCII pattern (>64 KB exercises DATA_FRAG fragmentation);
// otherwise publishes a short "espp interop N" string. Exits 0 after publishing
// `count` samples.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps_participant.hpp"

// std_msgs/msg/String, serialized via the reflection-driven cdr component:
// cdr::serialize<cdr::xcdr1> emits the ROS 2 / classic-CDR wire format
// (4-byte encapsulation header + CDR body) for any reflectable struct.
struct StringMsg {
  std::string data;
};

// Deterministic printable pattern shared by the pub, the sub, and the ROS 2
// generator in run_interop.sh, so large-payload reassembly can be checked
// byte-exact in both directions.
inline std::string make_pattern(std::size_t n) {
  std::string s(n, '\0');
  for (std::size_t i = 0; i < n; ++i) {
    s[i] = static_cast<char>('A' + (i % 26));
  }
  return s;
}

// The cdr component works in std::byte; the facade publish/on_sample API uses
// uint8_t spans - bridge the two views (same bytes, different value type).
inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  const char *topic = (argc > 1) ? argv[1] : "rt/chatter";
  const char *type = (argc > 2) ? argv[2] : "std_msgs::msg::dds_::String_";
  const bool reliable = (argc > 3) ? (std::atoi(argv[3]) != 0) : true;
  const int count = (argc > 4) ? std::atoi(argv[4]) : 30;
  const int period_ms = (argc > 5) ? std::atoi(argv[5]) : 200;
  const char *interface_ip = (argc > 6) ? argv[6] : ""; // "" -> auto-detect
  const std::size_t payload_bytes = (argc > 7) ? std::strtoul(argv[7], nullptr, 10) : 0;
  // Optional per-writer fragment size (bytes). 0 -> keep the facade default
  // (63000). Useful on hosts whose UDP max datagram is small (e.g. macOS caps
  // net.inet.udp.maxdgram at 9216) so fragments still fit one datagram.
  const int fragment_size = (argc > 8) ? std::atoi(argv[8]) : 0;

  espp::RtpsParticipant participant({
      .interface_address = interface_ip,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!participant.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  using Reliability = espp::RtpsParticipant::Reliability;
  espp::RtpsParticipant::WriterConfig wcfg{
      .topic = topic,
      .type_name = type,
      .reliability = reliable ? Reliability::RELIABLE : Reliability::BEST_EFFORT,
  };
  if (fragment_size > 0) {
    wcfg.fragment_size = static_cast<uint16_t>(fragment_size);
  }
  if (!participant.add_writer(wcfg)) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }
  std::printf("interop_pub: topic=%s type=%s reliable=%d count=%d payload_bytes=%zu\n", topic, type,
              reliable ? 1 : 0, count, payload_bytes);

  // Precompute the large deterministic pattern once (if in large-payload mode).
  const std::string pattern = payload_bytes > 0 ? make_pattern(payload_bytes) : std::string{};

  // Give SPDP/SEDP a moment to match before the first sample.
  std::this_thread::sleep_for(2s);

  int sent = 0;
  for (int i = 0; i < count; i++) {
    auto bytes = payload_bytes > 0
                     ? cdr::serialize<cdr::xcdr1>(StringMsg{pattern})
                     : cdr::serialize<cdr::xcdr1>(StringMsg{"espp interop " + std::to_string(i)});
    if (bytes && participant.publish(topic, u8_span(*bytes))) {
      sent++;
      std::printf("sent %d\n", sent);
      std::fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  // Let reliable retransmits drain before tearing down.
  std::this_thread::sleep_for(1s);
  std::printf("DONE sent=%d\n", sent);
  participant.stop();
  return sent == count ? 0 : 1;
}
