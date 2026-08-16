// In-process large-sample (DATA_FRAG) loopback: two espp::RtpsParticipant
// instances in one process exchange a >64 KB sample that must be fragmented on
// send and reassembled on receive, and the received bytes are verified to be
// byte-exact against the deterministic pattern that was published.
//
// This is the docker-free proof of Slice C's send-split + reassembly path
// (components/rtps/REFACTOR_PLAN.md). The engine transports the raw
// payload bytes unchanged, so we publish an arbitrary 200 KB byte ramp (well
// above the ~64 KB single-DATA limit) and assert the subscriber receives the
// identical bytes.
//
// Exits 0 when a byte-exact 200 KB sample is received within the deadline.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
constexpr std::size_t kPayloadBytes = 200000; // > 64 KB -> must fragment

std::vector<uint8_t> make_pattern(std::size_t n) {
  std::vector<uint8_t> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = static_cast<uint8_t>((i * 131u + 7u) & 0xFFu);
  }
  return v;
}
} // namespace

int main() {
  constexpr auto kDeadline = 40s;
  const char *topic = "frag_loopback";
  const char *type = "std_msgs::msg::dds_::ByteMultiArray_";
  const std::vector<uint8_t> pattern = make_pattern(kPayloadBytes);

  std::atomic<bool> ok{false};
  std::atomic<int> received{0};
  using Reliability = espp::RtpsParticipant::Reliability;

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::WARN});
  espp::RtpsParticipant sub({.log_level = espp::Logger::Verbosity::WARN});

  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  // Fragment size 8000: fits a single UDP datagram on any host (macOS caps
  // net.inet.udp.maxdgram at 9216), splitting the 200 KB sample into ~25
  // fragments. RELIABLE QoS + publishing a single sequence number (below) makes
  // delivery robust to any fragment loss: a lost fragment leaves the SN
  // unacknowledged, the writer re-heartbeats, the reader NACKs, and the whole SN
  // is retransmitted; the reader's single reassembly slot accumulates fragments
  // across retransmit rounds until complete. The receive side accepts ANY
  // peer-chosen fragment size.
  if (!pub.add_writer({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .fragment_size = 8000})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }
  if (!sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample = [&](std::span<const uint8_t> payload) {
                         received.fetch_add(1);
                         const bool match =
                             payload.size() == kPayloadBytes &&
                             std::equal(payload.begin(), payload.end(), pattern.begin());
                         if (match) {
                           ok.store(true);
                         } else {
                           // In this branch match is false by construction.
                           std::printf("received %zu bytes, byte-exact=0\n", payload.size());
                         }
                       }})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  // Give discovery a moment to match before the (large) sample.
  std::this_thread::sleep_for(3s);

  // Publish exactly ONE large sample (a single sequence number) and let RELIABLE
  // delivery drive it to completion: any dropped fragment is recovered by the
  // writer retransmitting the whole SN on NACK, and the reader's single
  // reassembly slot accumulates fragments across retransmit rounds without being
  // evicted by a newer SN. Re-publish only as a slow fallback if nothing arrived.
  int sent = 0;
  const auto start = std::chrono::steady_clock::now();
  while (!ok.load() && std::chrono::steady_clock::now() - start < kDeadline) {
    if (pub.publish(topic, std::span<const uint8_t>(pattern.data(), pattern.size()))) {
      sent++;
    }
    for (int i = 0; i < 120 && !ok.load(); ++i) { // wait up to ~12 s before re-publishing
      std::this_thread::sleep_for(100ms);
    }
  }

  std::printf("sent=%d received=%d byte_exact=%d\n", sent, received.load(), ok.load() ? 1 : 0);
  pub.stop();
  sub.stop();
  if (ok.load()) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL\n");
  return 1;
}
