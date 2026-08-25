// Shared-port deferred banded dispatch: a banded reader that gets NO dedicated
// port (dedicated ports disabled on the subscriber) must still receive every
// sample, in order, with its callback re-submitted to the transport pool at the
// reader's band instead of running inline on the receive worker.
//
// Phase 1 (deterministic, unit-level): proves the CORE guarantee the loopback
// below cannot - that a shared-port banded delivery is dispatched through the
// pool AT ITS BAND. Two DeferredDispatch instances (Low + High) enqueue a
// delivery each while both transport workers are blocked, so both drain jobs
// sit in the pool queue together; a single freed worker must then service the
// High drain BEFORE the Low drain (band-priority pop). If the drain were
// resubmitted at Normal (the regression this guards), the two jobs would be
// FIFO-ordered and Low (enqueued first) would run first - failing the test.
//
// Phase 2 (loopback): the publisher sends kTotal sequence-numbered samples
// (reliable); the test requires all of them, strictly in order, at the
// subscriber (delivery + per-reader ordering through the real wiring).
//
// Exits 0 on success.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "cdr.hpp"
#include "rtps/communication/EsppTransport.hpp"
#include "rtps_participant.hpp"

struct SeqMsg {
  uint32_t seq;
};

inline std::span<const uint8_t> u8_span(const std::vector<std::byte> &bytes) {
  return {reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()};
}

using namespace std::chrono_literals;

namespace {
// Expose the protected DeferredDispatch type for unit testing.
struct TestParticipant : espp::RtpsParticipant {
  using espp::RtpsParticipant::DeferredDispatch;
};
using DeferredDispatch = TestParticipant::DeferredDispatch;

void noop_rx(void *, const uint8_t *, std::size_t, rtps::Ip4Port_t, rtps::Ip4Port_t,
             const rtps::Ip4AddressBytes &) {}

// Returns 0 on success, 1 on failure.
int run_band_queue_jump_test() {
  rtps::EsppTransport transport(&noop_rx, nullptr);
  auto owner = std::make_shared<int>(0);

  auto low = std::make_shared<DeferredDispatch>();
  low->enabled = true;
  low->band = espp::QosBand::Low;
  low->transport = &transport;
  auto high = std::make_shared<DeferredDispatch>();
  high->enabled = true;
  high->band = espp::QosBand::High;
  high->transport = &transport;

  // Block BOTH workers with independent release flags so we can later free
  // exactly ONE and have it service both queued drains in band order.
  std::atomic<int> latched{0};
  std::atomic<bool> release_a{false};
  std::atomic<bool> release_b{false};
  const auto block = [&latched](std::atomic<bool> &rel) {
    latched.fetch_add(1);
    while (!rel.load()) {
      std::this_thread::sleep_for(1ms);
    }
  };
  if (!transport.submit([&] { block(release_a); }) ||
      !transport.submit([&] { block(release_b); })) {
    std::printf("FAIL: could not block the transport workers\n");
    return 1;
  }
  const auto latch_deadline = std::chrono::steady_clock::now() + 5s;
  while (latched.load() < 2 && std::chrono::steady_clock::now() < latch_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (latched.load() < 2) {
    std::printf("FAIL: workers never picked up the blockers\n");
    release_a = release_b = true;
    return 1;
  }

  // Enqueue Low FIRST, then High: both drain jobs are now queued at their
  // bands while the workers are busy.
  std::mutex order_mutex;
  std::vector<char> order;
  low->run_or_defer(
      [&] {
        std::lock_guard<std::mutex> lock(order_mutex);
        order.push_back('L');
      },
      owner);
  high->run_or_defer(
      [&] {
        std::lock_guard<std::mutex> lock(order_mutex);
        order.push_back('H');
      },
      owner);

  auto wait_for_count = [&](std::size_t n) {
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(order_mutex);
        if (order.size() >= n) {
          return true;
        }
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  };

  // Free ONE worker while both drains are queued: the queue-jump property is
  // that it services the higher-priority High drain FIRST, so the very first
  // delivery must be 'H' even though Low was enqueued first. (Two-phase, one
  // delivery per freed worker, so the assertion never depends on a single
  // worker draining both jobs within a timing window.)
  release_b = true;
  if (!wait_for_count(1)) {
    std::printf("FAIL: no banded drain ran after freeing the first worker\n");
    release_a = true;
    low->close();
    high->close();
    transport.stop();
    return 1;
  }
  {
    std::lock_guard<std::mutex> lock(order_mutex);
    if (order[0] != 'H') {
      std::printf("FAIL: High-band delivery did not overtake queued Low (first=%c)\n", order[0]);
      release_a = true;
      low->close();
      high->close();
      transport.stop();
      return 1;
    }
  }

  // Free the second worker: the remaining Low drain now runs.
  release_a = true;
  const bool both = wait_for_count(2);
  low->close();
  high->close();
  transport.stop();

  if (!both) {
    std::printf("FAIL: the queued Low drain never ran\n");
    return 1;
  }
  std::lock_guard<std::mutex> lock(order_mutex);
  if (order[0] != 'H' || order[1] != 'L') {
    std::printf("FAIL: unexpected delivery order (%c%c)\n", order[0], order[1]);
    return 1;
  }
  std::printf("queue-jump: High banded delivery overtook queued Low - PASS\n");
  return 0;
}
} // namespace

int main() {
  if (run_band_queue_jump_test() != 0) {
    return 1;
  }

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
