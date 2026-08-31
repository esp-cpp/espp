// Guaranteed transport submission (EsppTransport::submitGuaranteed): a writer's
// progress() poke MUST eventually run even when the bounded pool queue is full.
// A best-effort DATA has no heartbeat/acknack recovery path, so a silently
// rejected progress submission would strand the sample unsent forever (the
// regression this guards). On rejection the job is parked and a retry timer
// re-submits it once the pool has capacity - with NO further submissions.
//
// Unit-level and deterministic (mirrors rtps_deferred_recovery): both transport
// workers are blocked on a latch, the bounded queue is filled until submit()
// rejects, ONE guaranteed job is submitted (parked), the workers are released,
// and the job must run via the retry timer alone.
//
// Exits 0 on success.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

#include "rtps/communication/EsppTransport.hpp"

using namespace std::chrono_literals;

namespace {
void noop_rx(void *, const uint8_t *, std::size_t, rtps::Ip4Port_t, rtps::Ip4Port_t,
             const rtps::Ip4AddressBytes &) {}
} // namespace

int main() {
  rtps::EsppTransport transport(&noop_rx, nullptr);

  // Block both workers; wait until BOTH are actually running before filling the
  // queue (a blocker still queued when the fill completes would let a
  // late-waking worker service the guaranteed job early - see the same guard in
  // rtps_deferred_recovery).
  std::atomic<bool> release{false};
  std::atomic<int> latched{0};
  const auto blocker = [&release, &latched]() {
    latched.fetch_add(1);
    while (!release.load()) {
      std::this_thread::sleep_for(1ms);
    }
  };
  if (!transport.submit(blocker) || !transport.submit(blocker)) {
    std::printf("FAIL: could not block the transport workers\n");
    return 1;
  }
  const auto latch_deadline = std::chrono::steady_clock::now() + 5s;
  while (latched.load() < 2 && std::chrono::steady_clock::now() < latch_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (latched.load() < 2) {
    std::printf("FAIL: workers never picked up the blockers\n");
    release = true;
    return 1;
  }

  // Fill the bounded queue until a plain submit() is rejected.
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

  // Submit the SAME producer's guaranteed poke kExpected times while the pool
  // is saturated: each is rejected and parked. This is the lossless-coalescing
  // path - the pokes coalesce into one map entry with an owed count of
  // kExpected, and NONE may be dropped (each parked poke maps one-to-one to a
  // progress() run / one sample).
  constexpr int kExpected = 300; // > any old fixed cap; proves nothing is dropped
  std::atomic<int> ran{0};
  const int producer = 0; // arbitrary producer identity (a real caller passes its writer)
  for (int i = 0; i < kExpected; ++i) {
    transport.submitGuaranteed(
        &producer, [&ran]() { ran.fetch_add(1); }, espp::QosBand::High);
  }
  std::this_thread::sleep_for(100ms);
  if (ran.load() != 0) {
    std::printf("FAIL: guaranteed job ran while the pool was saturated?\n");
    release = true;
    return 1;
  }

  // Release the workers. NO further submissions: only the retry timer can get
  // the parked pokes into the pool once the fillers drain.
  release = true;
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (ran.load() < kExpected && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const int n = ran.load();

  transport.stop(); // cancels the retry timer, quiesces

  if (n != kExpected) {
    std::printf("FAIL: parked guaranteed pokes lost (ran=%d, expected=%d)\n", n, kExpected);
    return 1;
  }
  std::printf("PASS (all %d parked pokes recovered, none dropped)\n", kExpected);
  return 0;
}
