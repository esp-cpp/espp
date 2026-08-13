// Regression tests for two Phase 4 hazards that the plain loopback tests do NOT
// exercise (they never backlog the writer history):
//
//  1. No sample is skipped under backlog. A reliable writer that fills its
//     history must, on the dynamic (host) storage path, GROW and retain every
//     sample rather than dropping the oldest - and it must NOT advance its send
//     cursor as if a drop occurred. Regression guard for the
//     StatelessWriter/StatefulWriter cursor-advance vs dynamic-growth bug: the
//     writers used to advance m_nextSequenceNumberToSend on isFull(), skipping
//     the retained oldest samples so they were never sent.
//
//  2. fragment_size == 0 is rejected by add_writer (a zero fragment size makes
//     the fragmented send path fail while publish() would otherwise report
//     success and silently drop every large sample).
//
// Two participants in one process, RELIABLE QoS. Exits 0 iff both checks pass.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <set>
#include <span>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
using Reliability = espp::RtpsParticipant::Reliability;

// 8-byte payload: 4-byte CDR_LE encapsulation header + a little-endian uint32
// sequence value. The engine transports the bytes verbatim; we only need to
// recover the sequence number on the receive side.
std::vector<uint8_t> encode_seq(uint32_t seq) {
  std::vector<uint8_t> b{0x00, 0x01, 0x00, 0x00, 0, 0, 0, 0};
  b[4] = static_cast<uint8_t>(seq & 0xFF);
  b[5] = static_cast<uint8_t>((seq >> 8) & 0xFF);
  b[6] = static_cast<uint8_t>((seq >> 16) & 0xFF);
  b[7] = static_cast<uint8_t>((seq >> 24) & 0xFF);
  return b;
}
uint32_t decode_seq(std::span<const uint8_t> p) {
  return static_cast<uint32_t>(p[4]) | (static_cast<uint32_t>(p[5]) << 8) |
         (static_cast<uint32_t>(p[6]) << 16) | (static_cast<uint32_t>(p[7]) << 24);
}
} // namespace

int main() {
  const char *type = "std_msgs::msg::dds_::UInt32_";

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant sub({.log_level = espp::Logger::Verbosity::WARN});
  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }

  // --- Check 2: fragment_size == 0 must be rejected. ---
  if (pub.add_writer({.topic = "cfg_zero_frag", .type_name = type, .fragment_size = 0})) {
    std::printf("FAIL: add_writer accepted fragment_size == 0\n");
    return 1;
  }

  // --- Check 1: no sample skipped under backlog. ---
  constexpr int N = 300; // >> host history depth, so the history backlogs + grows
  const char *topic = "backlog";
  std::mutex m;
  std::set<uint32_t> got;

  if (!pub.add_writer({.topic = topic, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }
  if (!sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = [&](std::span<const uint8_t> payload) {
                         if (payload.size() < 8) {
                           return;
                         }
                         std::lock_guard<std::mutex> lk(m);
                         got.insert(decode_seq(payload));
                       }})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  std::this_thread::sleep_for(2s); // let SEDP match

  // Publish N sequence values fast enough that the reliable writer's history
  // fills before delivery drains it - forcing the grow/backlog path.
  for (int i = 0; i < N; ++i) {
    pub.publish(topic, encode_seq(static_cast<uint32_t>(i)));
    std::this_thread::sleep_for(2ms);
  }

  // Wait for reliable delivery to drain.
  const auto deadline = std::chrono::steady_clock::now() + 30s;
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(m);
      if (static_cast<int>(got.size()) >= N) {
        break;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // Verify EVERY sequence 0..N-1 arrived (a skipped/never-sent sample shows up as
  // a gap - the exact symptom of the cursor-advance bug).
  std::size_t count;
  std::vector<uint32_t> missing;
  {
    std::lock_guard<std::mutex> lk(m);
    count = got.size();
    for (uint32_t i = 0; i < static_cast<uint32_t>(N); ++i) {
      if (got.find(i) == got.end()) {
        missing.push_back(i);
      }
    }
  }
  pub.stop();
  sub.stop();

  std::printf("published=%d received=%zu missing=%zu\n", N, count, missing.size());
  if (missing.empty()) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %zu samples never delivered (first missing seq=%u)\n", missing.size(),
              missing.front());
  return 1;
}
