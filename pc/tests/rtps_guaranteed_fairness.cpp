// Fairness / eventual-run for the guaranteed-submission retry path
// (EsppTransport::submitGuaranteed): under SUSTAINED overload, every parked
// producer must eventually be admitted to the pool - regardless of its band and
// of how many higher-band producers are continuously owed. This is the
// regression pair that broke twice during review:
//  - drain-one-key-to-exhaustion starved every later key (pointer order), and
//  - band-sorted admission starved lower bands forever when the higher bands
//    could consume every freed slot each retry tick.
// The admission stage must be band-agnostic round-robin; band priority belongs
// to the pool's banded queues AFTER admission.
//
// Deterministic shape (mirrors rtps_guaranteed_submit): both transport workers
// are latched, the bounded queue is filled with SLOW jobs, then owed runs are
// parked for 32 High keys (100 each) + 2 Normal keys (10 each) + 1 Low key (1).
// Releasing the latch creates sustained overload: workers free only ~8 slots
// per 20 ms retry tick while the High backlog (3200 slow jobs, ~6+ s of work)
// refills them. Band-sorted admission would keep the Low key saturated for the
// whole backlog (deterministically past the deadline); round-robin admission
// must admit it within one rotor revolution (< 1 s).
//
// Once the Low job has run, the remaining jobs switch to no-ops (fast drain)
// and the test verifies the lossless contract: every owed run for every key
// executed exactly once.
//
// Exits 0 on success.

#include <array>
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

  // Latch both workers so the queue can be saturated deterministically.
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

  // While the Low producer is still starved, every job is SLOW (sleep) so the
  // workers can only free a handful of queue slots per retry tick - sustained
  // overload. After the Low job has run the fairness property is proven and
  // the remaining backlog switches to no-ops so the test finishes quickly.
  std::atomic<bool> fast_drain{false};
  const auto slow_work = [&fast_drain]() {
    if (!fast_drain.load()) {
      std::this_thread::sleep_for(4ms);
    }
  };

  // Fill the bounded queue with SLOW jobs until submit() rejects.
  int fillers = 0;
  while (transport.submit(slow_work) && fillers < 100000) {
    ++fillers;
  }
  if (fillers >= 100000) {
    std::printf("FAIL: transport queue never rejected (unbounded?)\n");
    release = true;
    return 1;
  }
  std::printf("queue saturated after %d slow filler jobs\n", fillers);

  // Park the competing producers (all rejected -> coalesced owed counts).
  // Distinct key identities come from distinct array elements.
  constexpr int kHighKeys = 32;
  constexpr int kHighOwedEach = 100; // 3200 slow jobs ~= 6.4 s of backlog at 2 workers
  constexpr int kNormalKeys = 2;
  constexpr int kNormalOwedEach = 10;
  static std::array<int, kHighKeys + kNormalKeys + 1> keys{};

  std::atomic<int> high_ran{0};
  std::atomic<int> normal_ran{0};
  std::atomic<bool> low_ran{false};

  for (int k = 0; k < kHighKeys; ++k) {
    for (int i = 0; i < kHighOwedEach; ++i) {
      transport.submitGuaranteed(
          &keys[k],
          [&high_ran, &slow_work]() {
            slow_work();
            high_ran.fetch_add(1);
          },
          espp::QosBand::High);
    }
  }
  for (int k = 0; k < kNormalKeys; ++k) {
    for (int i = 0; i < kNormalOwedEach; ++i) {
      transport.submitGuaranteed(
          &keys[kHighKeys + k],
          [&normal_ran, &slow_work]() {
            slow_work();
            normal_ran.fetch_add(1);
          },
          espp::QosBand::Normal);
    }
  }
  transport.submitGuaranteed(
      &keys[kHighKeys + kNormalKeys], [&low_ran]() { low_ran.store(true); }, espp::QosBand::Low);

  // Release the workers: sustained overload begins. The single Low owed run
  // must be admitted while the High backlog is still deep.
  const auto start = std::chrono::steady_clock::now();
  release = true;
  const auto fairness_deadline = start + 5s;
  while (!low_ran.load() && std::chrono::steady_clock::now() < fairness_deadline) {
    std::this_thread::sleep_for(5ms);
  }
  if (!low_ran.load()) {
    std::printf("FAIL: Low-band producer starved (high_ran=%d of %d while Low never admitted)\n",
                high_ran.load(), kHighKeys * kHighOwedEach);
    fast_drain = true;
    transport.stop();
    return 1;
  }
  const auto low_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - start)
                          .count();
  std::printf("Low admitted after %lld ms (high backlog remaining: %d)\n",
              static_cast<long long>(low_ms), kHighKeys * kHighOwedEach - high_ran.load());

  // Lossless: with the fairness property proven, drain the remaining backlog
  // fast and require every owed run to have executed exactly once.
  fast_drain = true;
  const int high_expected = kHighKeys * kHighOwedEach;
  const int normal_expected = kNormalKeys * kNormalOwedEach;
  const auto drain_deadline = std::chrono::steady_clock::now() + 20s;
  while ((high_ran.load() < high_expected || normal_ran.load() < normal_expected) &&
         std::chrono::steady_clock::now() < drain_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  const int h = high_ran.load();
  const int n = normal_ran.load();

  transport.stop();

  if (h != high_expected || n != normal_expected) {
    std::printf("FAIL: owed runs lost (high=%d/%d, normal=%d/%d)\n", h, high_expected, n,
                normal_expected);
    return 1;
  }
  std::printf("PASS (Low admitted under sustained High overload; all %d owed runs executed)\n",
              high_expected + normal_expected + 1);
  return 0;
}
