// RTPS interop endpoint churn: a peer's publisher comes and goes, and the
// matched writer must go with it.
//
// A DDS peer that deletes a publisher, or leaves the domain, tells the others
// with a dispose (RTPS 2.5 8.5.4.3 / 9.6.3.9: a DATA whose inline QoS carries
// PID_STATUS_INFO and PID_KEY_HASH). This subscriber watches the count of
// remote writers matched to its reader (RtpsParticipant::matched_writers) and
// counts every transition from matched to unmatched. The ROS 2 side
// (interop/ros2_churn_publisher.py) creates, uses and destroys a publisher in
// rounds a few seconds long; the only way to see one unmatch per round within
// the run is to act on the disposes - a peer dropped on lease expiry alone
// (20 s Fast DDS, 10 s Cyclone DDS) would yield at most one or two.
//
// Usage: rtps_interop_churn [topic] [type] [required_unmatches] [timeout_s]
// [interface_ip]
// Exits 0 once `required_unmatches` matched->unmatched transitions were seen
// within `timeout_s`, each preceded by at least one received sample (the match
// was real); 1 otherwise.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "cdr.hpp"
#include "rtps_participant.hpp"

struct StringMsg {
  std::string data;
};

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  const char *topic = (argc > 1) ? argv[1] : "rt/churn";
  const char *type = (argc > 2) ? argv[2] : "std_msgs::msg::dds_::String_";
  const int required = (argc > 3) ? std::atoi(argv[3]) : 3;
  const int timeout_s = (argc > 4) ? std::atoi(argv[4]) : 40;
  const char *interface_ip = (argc > 5) ? argv[5] : ""; // "" -> auto-detect
  if (required <= 0 || timeout_s <= 0) {
    std::printf("FAIL: required_unmatches and timeout_s must be > 0\n");
    return 1;
  }

  std::atomic<int> received{0};
  const auto start = std::chrono::steady_clock::now();
  const auto elapsed_s = [&start]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  };

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
          .reliability = Reliability::RELIABLE,
          .on_sample =
              [&received](std::span<const uint8_t> cdr_payload) {
                if (cdr::deserialize<StringMsg>(std::as_bytes(cdr_payload))) {
                  received.fetch_add(1);
                }
              },
      })) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }
  std::printf("interop_churn: topic=%s type=%s required_unmatches=%d timeout=%ds\n", topic, type,
              required, timeout_s);

  // Poll the matched-writer count; a transition from >0 to 0 is one unmatch,
  // and it only counts when that match delivered something.
  std::size_t last_matched = 0;
  int received_at_match = 0;
  int unmatches = 0;
  int empty_unmatches = 0;
  while (elapsed_s() < timeout_s && unmatches < required) {
    const std::size_t matched = participant.matched_writers(topic);
    if (matched != last_matched) {
      std::printf("t=%.1fs matched writers %zu -> %zu (received so far %d)\n", elapsed_s(),
                  last_matched, matched, received.load());
      std::fflush(stdout);
      if (last_matched == 0 && matched > 0) {
        received_at_match = received.load();
      } else if (last_matched > 0 && matched == 0) {
        if (received.load() > received_at_match) {
          ++unmatches;
        } else {
          ++empty_unmatches;
        }
      }
      last_matched = matched;
    }
    std::this_thread::sleep_for(50ms);
  }

  const bool ok = unmatches >= required;
  std::printf("%s: unmatches=%d (required %d, %d without samples), received=%d, %.1fs\n",
              ok ? "PASS" : "FAIL", unmatches, required, empty_unmatches, received.load(),
              elapsed_s());
  participant.stop();
  return ok ? 0 : 1;
}
