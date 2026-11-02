// RTPS interop subscriber (Phase 1 of components/rtps/REFACTOR_PLAN.md).
//
// Exercises the espp::RtpsParticipant facade end-to-end: subscribes to CDR string
// samples (deserialized with the reflection-driven cdr::deserialize) published by an external DDS
// peer (FastDDS or a ROS 2 node via rmw_fastrtps). Defaults follow the ROS 2 conventions for
// std_msgs/String on /chatter.
//
// Usage: rtps_interop_sub [topic] [type] [reliable(0|1)] [required] [timeout_s]
// [interface_ip] [payload_bytes] [band] [count_after_s]
// When payload_bytes > 0, each received String is verified byte-exact against the
// deterministic payload_bytes-long pattern (proving fragmented >64 KB samples are
// reassembled correctly); only byte-exact receptions count toward `required`.
// band (0=Critical 1=High 2=Normal 3=Low; default 2=Normal): the reader's
// espp::QosBand. A non-Normal band gives the reader a DEDICATED unicast port
// announced via its SEDP per-endpoint unicast locator - the interop matrix uses
// this to prove a FastDDS/ROS 2 peer honors that locator and delivers the
// topic's traffic to the dedicated port.
// count_after_s (default 0, must be >= 0 - a negative value is a usage error
// and exits 1): samples that arrive earlier than this many seconds after start
// are reported (and, in large-payload mode, still verified byte-exact) but
// NOT counted toward `required`. The interop
// matrix uses this as a liveliness gate: a peer whose participant we dropped
// (e.g. its lease expired because its re-announcements were not delivered)
// stops being matched, so nothing counts after the lease - only samples that
// keep arriving past that point prove the peer stayed alive.
// Exits 0 once `required` (counted) samples arrive within `timeout_s`.

#include <atomic>
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

// Deterministic printable pattern shared with rtps_interop_pub and the
// ROS 2 generator in run_interop.sh (see that file).
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
  const int required = (argc > 4) ? std::atoi(argv[4]) : 5;
  const int timeout_s = (argc > 5) ? std::atoi(argv[5]) : 30;
  const char *interface_ip = (argc > 6) ? argv[6] : ""; // "" -> auto-detect
  const std::size_t payload_bytes = (argc > 7) ? std::strtoul(argv[7], nullptr, 10) : 0;
  const int band_arg = (argc > 8) ? std::atoi(argv[8]) : static_cast<int>(espp::QosBand::Normal);
  const int count_after_s = (argc > 9) ? std::atoi(argv[9]) : 0;
  if (count_after_s < 0) {
    std::printf("FAIL: count_after_s must be >= 0, got %d\n", count_after_s);
    return 1;
  }
  // Validate before casting: an out-of-range value would index band arrays.
  if (band_arg < static_cast<int>(espp::QosBand::Critical) ||
      band_arg > static_cast<int>(espp::QosBand::Low)) {
    std::printf("FAIL: band must be 0..%d (0=Critical 1=High 2=Normal 3=Low), got %d\n",
                static_cast<int>(espp::QosBand::Low), band_arg);
    return 1;
  }
  const auto band = static_cast<espp::QosBand>(band_arg);
  const std::string expected = payload_bytes > 0 ? make_pattern(payload_bytes) : std::string{};

  std::atomic<int> received{0};
  std::atomic<int> early{0}; // arrived before count_after_s; reported, not counted
  const auto start = std::chrono::steady_clock::now();
  const auto count_from = start + std::chrono::seconds(count_after_s);

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
              [&received, &early, count_from, payload_bytes,
               &expected](std::span<const uint8_t> cdr_payload) {
                auto msg = cdr::deserialize<StringMsg>(std::as_bytes(cdr_payload));
                if (!msg) {
                  return;
                }
                // Early samples (before count_after_s) are verified and
                // reported exactly like counted ones; only the count differs.
                const bool counts = std::chrono::steady_clock::now() >= count_from;
                if (payload_bytes > 0) {
                  // Large-payload mode: only a byte-exact reassembly counts.
                  const bool exact = msg->data.size() == payload_bytes && msg->data == expected;
                  if (!exact) {
                    std::printf("received %zu bytes, byte-exact=0 (expected %zu)\n",
                                msg->data.size(), payload_bytes);
                    std::fflush(stdout);
                    return;
                  }
                  if (!counts) {
                    const int n = early.fetch_add(1) + 1;
                    std::printf("received (not counted yet) %d: %zu bytes byte-exact\n", n,
                                msg->data.size());
                    std::fflush(stdout);
                    return;
                  }
                  const int n = received.fetch_add(1) + 1;
                  std::printf("received %d: %zu bytes byte-exact\n", n, msg->data.size());
                  std::fflush(stdout);
                } else {
                  if (!counts) {
                    const int n = early.fetch_add(1) + 1;
                    std::printf("received (not counted yet) %d: '%s'\n", n, msg->data.c_str());
                    std::fflush(stdout);
                    return;
                  }
                  const int n = received.fetch_add(1) + 1;
                  std::printf("received %d: '%s'\n", n, msg->data.c_str());
                  std::fflush(stdout);
                }
              },
          .band = band,
      })) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }
  std::printf("interop_sub: topic=%s type=%s reliable=%d required=%d timeout=%ds band=%d "
              "count_after=%ds\n",
              topic, type, reliable ? 1 : 0, required, timeout_s, band_arg, count_after_s);

  while (received.load() < required &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout_s)) {
    std::this_thread::sleep_for(100ms);
  }

  const int n = received.load();
  std::printf("%s received=%d required=%d (not counted, before %ds: %d)\n",
              n >= required ? "PASS" : "FAIL", n, required, count_after_s, early.load());
  participant.stop();
  return n >= required ? 0 : 1;
}
