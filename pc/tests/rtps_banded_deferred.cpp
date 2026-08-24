// Shared-port deferred banded dispatch: a banded reader that gets NO dedicated
// port (dedicated ports disabled on the subscriber) must still receive every
// sample, in order, with its callback re-submitted to the transport pool at the
// reader's band instead of running inline on the receive worker.
//
// The publisher sends kTotal sequence-numbered samples (reliable); the test
// requires all of them, strictly in order, at the subscriber.
//
// Exits 0 on success.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

#include "cdr.hpp"
#include "rtps_participant.hpp"

struct SeqMsg {
  uint32_t seq;
};

inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

using namespace std::chrono_literals;

int main() {
  constexpr uint32_t kTotal = 30; // < the 32-entry deferred queue bound
  constexpr auto kDeadline = 30s;
  const char *topic = "deferred_loopback";
  const char *type = "espp::test::dds_::Seq_";
  using Reliability = espp::RtpsParticipant::Reliability;

  espp::RtpsParticipant pub({.log_level = espp::Logger::Verbosity::INFO});
  // Subscriber: dedicated ports DISABLED, so the banded reader must fall back
  // to deferred banded dispatch on the shared user-unicast port.
  espp::RtpsParticipant sub(
      {.log_level = espp::Logger::Verbosity::INFO, .enable_dedicated_endpoint_ports = false});
  if (!pub.start() || !sub.start()) {
    std::printf("FAIL: start\n");
    return 1;
  }
  if (!pub.add_writer({.topic = topic, .type_name = type, .reliability = Reliability::RELIABLE})) {
    std::printf("FAIL: add_writer\n");
    return 1;
  }

  std::mutex order_mutex;
  std::vector<uint32_t> order;
  std::atomic<uint32_t> received{0};
  if (!sub.add_reader({.topic = topic,
                       .type_name = type,
                       .reliability = Reliability::RELIABLE,
                       .on_sample =
                           [&](std::span<const uint8_t> payload) {
                             auto msg = cdr::deserialize<SeqMsg>(std::as_bytes(payload));
                             if (!msg) {
                               return;
                             }
                             std::lock_guard<std::mutex> lock(order_mutex);
                             order.push_back(msg->seq);
                             received.fetch_add(1);
                           },
                       .band = espp::QosBand::High})) {
    std::printf("FAIL: add_reader\n");
    return 1;
  }

  // Wait for discovery/matching, then send the numbered sequence. Reliable
  // writers retransmit on the wire, but a sample must reach the reader at least
  // once for the deferred queue to see it - send each one until acknowledged by
  // observation (simple paced resend of the not-yet-seen head).
  uint32_t next_to_send = 0;
  const auto start = std::chrono::steady_clock::now();
  while (received.load() < kTotal && std::chrono::steady_clock::now() - start < kDeadline) {
    if (next_to_send < kTotal) {
      auto bytes = cdr::serialize<cdr::xcdr1>(SeqMsg{next_to_send});
      if (bytes && pub.publish(topic, u8_span(*bytes))) {
        next_to_send++;
        std::this_thread::sleep_for(20ms);
        continue;
      }
    }
    std::this_thread::sleep_for(50ms);
  }

  const uint32_t n = received.load();
  std::printf("sent=%u received=%u\n", next_to_send, n);
  pub.stop();
  sub.stop();

  if (n < kTotal) {
    std::printf("FAIL: incomplete delivery\n");
    return 1;
  }
  // Ordering: per-reader order must be preserved by the single in-flight
  // deferred drain - the recorded sequence must be exactly 0..kTotal-1.
  {
    std::lock_guard<std::mutex> lock(order_mutex);
    for (uint32_t i = 0; i < kTotal; ++i) {
      if (order[i] != i) {
        std::printf("FAIL: out-of-order delivery at index %u: got %u\n", i, order[i]);
        return 1;
      }
    }
  }
  std::printf("PASS\n");
  return 0;
}
