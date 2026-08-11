// RTPS interop subscriber (Phase 1 of components/rtps_embedded/REFACTOR_PLAN.md).
//
// Exercises the espp::RtpsParticipant facade end-to-end: subscribes to CDR string
// samples (deserialized with espp::CdrReader) published by an external DDS peer
// (FastDDS or a ROS 2 node via rmw_fastrtps). Defaults follow the ROS 2
// conventions for std_msgs/String on /chatter.
//
// Usage: rtps_embedded_interop_sub [topic] [type] [reliable(0|1)] [required] [timeout_s]
// [interface_ip] Exits 0 once `required` samples arrive within `timeout_s`.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  const char *topic = (argc > 1) ? argv[1] : "rt/chatter";
  const char *type = (argc > 2) ? argv[2] : "std_msgs::msg::dds_::String_";
  const bool reliable = (argc > 3) ? (std::atoi(argv[3]) != 0) : true;
  const int required = (argc > 4) ? std::atoi(argv[4]) : 5;
  const int timeout_s = (argc > 5) ? std::atoi(argv[5]) : 30;
  const char *interface_ip = (argc > 6) ? argv[6] : ""; // "" -> auto-detect

  std::atomic<int> received{0};

  espp::RtpsParticipant participant({
      .interface_address = interface_ip,
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!participant.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  using Reliability = espp::RtpsParticipant::Reliability;
  if (!participant.add_reader({
          .topic = topic,
          .type_name = type,
          .reliability = reliable ? Reliability::RELIABLE : Reliability::BEST_EFFORT,
          .on_sample =
              [&received](std::span<const uint8_t> cdr_payload) {
                espp::CdrReader reader(cdr_payload); // expects the encapsulation header
                std::string text;
                if (reader.read_string(text)) {
                  const int n = received.fetch_add(1) + 1;
                  std::printf("received %d: '%s'\n", n, text.c_str());
                  std::fflush(stdout);
                }
              },
      })) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }
  std::printf("interop_sub: topic=%s type=%s reliable=%d required=%d timeout=%ds\n", topic, type,
              reliable ? 1 : 0, required, timeout_s);

  const auto start = std::chrono::steady_clock::now();
  while (received.load() < required &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout_s)) {
    std::this_thread::sleep_for(100ms);
  }

  const int n = received.load();
  std::printf("%s received=%d required=%d\n", n >= required ? "PASS" : "FAIL", n, required);
  participant.stop();
  return n >= required ? 0 : 1;
}
