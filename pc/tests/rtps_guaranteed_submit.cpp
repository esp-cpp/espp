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

  // Submit ONE guaranteed job: the pool rejects it right now, so it must be
  // parked (not run) rather than silently dropped.
  std::atomic<int> ran{0};
  transport.submitGuaranteed([&ran]() { ran.fetch_add(1); }, espp::QosBand::High);
  std::this_thread::sleep_for(100ms);
  if (ran.load() != 0) {
    std::printf("FAIL: guaranteed job ran while the pool was saturated?\n");
    release = true;
    return 1;
  }

  // Release the workers. NO further submissions: only the retry timer can get
  // the parked job into the pool once the fillers drain.
  release = true;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (ran.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const int n = ran.load();

  transport.stop(); // cancels the retry timer, quiesces

  if (n != 1) {
    std::printf("FAIL: parked guaranteed job never recovered (ran=%d)\n", n);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
