// Regression test for DATA_FRAG reassembly with a NON-exact final fragment.
//
// The existing rtps_facade_frag test uses 200000 bytes at fragment_size 8000,
// which divides evenly (25 x 8000) - so its last fragment is full and the
// short/padded-last-fragment reassembly path is never exercised locally (only
// the docker ROS 2 leg hits it). Here we deliberately pick payload sizes that do
// NOT divide evenly by the fragment size, so the final fragment is short. This
// guards the reader reassembly's fragment-length handling (Reader::newFragment)
// against regressions - the clamp/validate logic that makes FastDDS/ROS 2
// large-sample interop work.
//
// Both cases use fragment_size 8000: on macOS a UDP datagram is capped at 9216
// bytes (net.inet.udp.maxdgram), so a single fragment must stay under that; the
// large default 63000 is a LAN/Linux setting. The payload sizes are chosen so
// the sample still exceeds the single-DATA limit (~65451, forcing fragmentation)
// and does NOT divide evenly by the fragment size (forcing a short final
// fragment):
//   - 70003 B @ 8000 -> 9 fragments (8x8000 + 6003), short last.
//   - 205003 B @ 8000 -> 26 fragments (25x8000 + 5003), short last.
//
// Exits 0 iff every case is received byte-exact.

#include <algorithm>
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
using Reliability = espp::RtpsParticipant::Reliability;

std::vector<uint8_t> make_pattern(std::size_t n) {
  std::vector<uint8_t> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = static_cast<uint8_t>((i * 131u + 7u) & 0xFFu);
  }
  return v;
}

// Run one payload-size / fragment-size case through an in-process reliable
// loopback and return true iff the sample is received byte-exact.
bool run_case(std::size_t payload_bytes, uint16_t fragment_size, const char *topic) {
  const std::vector<uint8_t> pattern = make_pattern(payload_bytes);
  std::atomic<bool> ok{false};
  const char *type = "std_msgs::msg::dds_::ByteMultiArray_";

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant sub({.log_level = espp::Logger::Verbosity::WARN});
  if (!pub.start() || !sub.start()) {
    return false;
  }
  if (!pub.add_writer({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .fragment_size = fragment_size})) {
    return false;
  }
  if (!sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = [&](std::span<const uint8_t> payload) {
                         if (payload.size() == pattern.size() &&
                             std::equal(payload.begin(), payload.end(), pattern.begin())) {
                           ok.store(true);
                         }
                       }})) {
    return false;
  }

  std::this_thread::sleep_for(2s); // discovery
  const auto deadline = std::chrono::steady_clock::now() + 30s;
  while (!ok.load() && std::chrono::steady_clock::now() < deadline) {
    pub.publish(topic, std::span<const uint8_t>(pattern.data(), pattern.size()));
    for (int i = 0; i < 100 && !ok.load(); ++i) {
      std::this_thread::sleep_for(100ms);
    }
  }
  const bool result = ok.load();
  pub.stop();
  sub.stop();
  std::printf("case payload=%zu frag=%u byte_exact=%d\n", payload_bytes, fragment_size,
              result ? 1 : 0);
  return result;
}
} // namespace

int main() {
  bool ok = true;
  ok &= run_case(70003, 8000, "frag_sizes_a");  // 9 fragments, short last
  ok &= run_case(205003, 8000, "frag_sizes_b"); // 26 fragments, short last
  if (ok) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
