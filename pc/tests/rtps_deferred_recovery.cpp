// Deferred-dispatch arm recovery: when the transport pool REJECTS the drain
// arm (workers busy + bounded queue full), a queued - possibly lone/last -
// delivery must still be delivered without any further traffic: the dispatcher
// flags the failed arm and its retry timer re-arms the drain once the pool has
// capacity again. Before the fix, the delivery stayed queued forever (a
// reliable reader had already acked the sample, so nothing would ever
// retransmit it).
//
// Unit-level and fully deterministic: both transport workers are blocked on a
// latch, the bounded queue is filled until submit() rejects, ONE delivery is
// enqueued (arm rejected), the workers are released, and the delivery must
// arrive with NO further run_or_defer() calls.
//
// Exits 0 on success.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include "rtps/communication/EsppTransport.hpp"
#include "rtps_participant.hpp"

using namespace std::chrono_literals;

namespace {
// Expose the protected DeferredDispatch type for unit testing.
struct TestParticipant : espp::RtpsParticipant {
  using espp::RtpsParticipant::DeferredDispatch;
};
using DeferredDispatch = TestParticipant::DeferredDispatch;

void noop_rx(void *, const uint8_t *, std::size_t, rtps::Ip4Port_t, rtps::Ip4Port_t,
             const rtps::Ip4AddressBytes &) {}
} // namespace

int main() {
  rtps::EsppTransport transport(&noop_rx, nullptr);

  // The owning context stand-in: drain jobs and the retry timer capture it.
  auto owner = std::make_shared<int>(0);
  auto dispatch = std::make_shared<DeferredDispatch>();
  dispatch->enabled = true;
  dispatch->band = espp::QosBand::High;
  dispatch->transport = &transport;

  // Saturate the pool: block both workers, then fill the bounded queue until
  // submissions are rejected.
  std::atomic<bool> release{false};
  const auto blocker = [&release]() {
    while (!release.load()) {
      std::this_thread::sleep_for(1ms);
    }
  };
  if (!transport.submit(blocker) || !transport.submit(blocker)) {
    std::printf("FAIL: could not block the transport workers\n");
    return 1;
  }
  int fillers = 0;
  while (transport.submit([]() {}) && fillers < 100000) {
    ++fillers;
  }
  if (fillers >= 100000) {
    std::printf("FAIL: transport queue never rejected (unbounded?)\n");
    release = true;
    return 1;
  }
  std::printf("queue saturated after %d filler jobs\n", fillers);

  // Enqueue ONE delivery: the drain arm must be rejected right now.
  std::atomic<int> delivered{0};
  dispatch->run_or_defer([&delivered]() { delivered.fetch_add(1); }, owner);
  std::this_thread::sleep_for(100ms);
  if (delivered.load() != 0) {
    std::printf("FAIL: delivery ran while the pool was saturated?\n");
    release = true;
    return 1;
  }

  // Release the workers. NO further traffic: only the retry timer can re-arm
  // the drain - the queued lone delivery must arrive.
  release = true;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (delivered.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const int n = delivered.load();

  // Lifetime discipline: quiesce before dropping references / stopping.
  dispatch->close();
  transport.stop();

  if (n != 1) {
    std::printf("FAIL: lone queued delivery never recovered (delivered=%d)\n", n);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
