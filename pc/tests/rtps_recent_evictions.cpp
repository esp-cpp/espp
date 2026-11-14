// The admission rule behind Participant::evictUnusedRemoteParticipant on a
// domain that stays bigger than the remote-participant pool (RecentEvictions):
// a peer evicted lately is held off, a former evictee past its hold-off waits
// for the per-period cooldown, a peer never evicted is served at once. It
// used to be one eviction per period for anyone: the peers that did not fit
// re-announced every period and took turns displacing each other, and a late
// publisher the device needed lost every round for 90 s in the QEMU suite
// (pool_full_evicts_unused_for_late_joiner). This pins the rule and the ring.

#include <chrono>
#include <cstdio>

#include "rtps/discovery/RecentEvictions.hpp"

namespace {
using namespace std::chrono_literals;
using Ring = rtps::RecentEvictions<4>;
using Gate = Ring::Gate;
using tp = Ring::time_point;

rtps::GuidPrefix_t prefix(uint8_t tag) {
  rtps::GuidPrefix_t p{};
  p.id.fill(tag);
  return p;
}

const char *name(Gate g) {
  switch (g) {
  case Gate::Proceed:
    return "Proceed";
  case Gate::HeldOff:
    return "HeldOff";
  case Gate::Cooldown:
    return "Cooldown";
  }
  return "?";
}

bool expect(Gate got, Gate want, const char *what) {
  if (got != want) {
    std::printf("FAIL: %s: %s, expected %s\n", what, name(got), name(want));
    return false;
  }
  return true;
}
} // namespace

int main() {
  bool ok = true;
  const auto t0 = tp{} + 1000s; // a default (zero) time_point means "never"
  constexpr auto cooldown = 2000ms;
  constexpr auto holdoff = 10000ms;
  const auto never = tp{};
  Ring ring;
  const auto a = prefix(0xA1);
  const auto b = prefix(0xB2);
  const auto c = prefix(0xC3);

  // unknown peers proceed whatever the cooldown state
  ok = expect(ring.gate(a, t0, never, cooldown, holdoff), Gate::Proceed,
              "unknown, no eviction yet") &&
       ok;
  ok = expect(ring.gate(a, t0, t0 - 100ms, cooldown, holdoff), Gate::Proceed,
              "unknown peer during the cooldown is still served") &&
       ok;
  if (ring.lastEviction(a)) {
    std::printf("FAIL: an unknown peer has an eviction time\n");
    ok = false;
  }

  // a evicted at t0: held off for holdoff, then cooldown-bound, then free
  ring.note(a, t0);
  if (!ring.lastEviction(a) || *ring.lastEviction(a) != t0) {
    std::printf("FAIL: the eviction time of a is not remembered\n");
    ok = false;
  }
  ok = expect(ring.gate(a, t0 + 1s, t0, cooldown, holdoff), Gate::HeldOff, "a one second after") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff - 1ms, t0, cooldown, holdoff), Gate::HeldOff,
              "a just before the hold-off ends") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff, t0 + holdoff - 500ms, cooldown, holdoff), Gate::Cooldown,
              "a past the hold-off, an eviction 500 ms ago") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff, t0 + holdoff - cooldown, cooldown, holdoff), Gate::Proceed,
              "a past the hold-off, the cooldown just over") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff, never, cooldown, holdoff), Gate::Proceed,
              "a past the hold-off, no eviction ever") &&
       ok;
  // meanwhile b, never evicted, is served even right after a's eviction
  ok = expect(ring.gate(b, t0 + 1ms, t0, cooldown, holdoff), Gate::Proceed,
              "b right after a's eviction") &&
       ok;

  // a second eviction of a refreshes its entry rather than adding one
  ring.note(a, t0 + 30s);
  ok = expect(ring.gate(a, t0 + 31s, t0 + 30s, cooldown, holdoff), Gate::HeldOff,
              "a held off again") &&
       ok;
  ring.note(b, t0 + 31s);
  ring.note(c, t0 + 32s);
  // the ring (4 entries) still knows a, b, c; one more distinct peer than it
  // holds overwrites the OLDEST slot (a, noted first) - not the refreshed one
  // in time order, but the ring is a ring: it rolls by slot, which is what a
  // bounded, allocation-free memory affords. Pin that it keeps the newest.
  ring.note(prefix(0xD4), t0 + 33s);
  ring.note(prefix(0xE5), t0 + 34s);
  if (ring.lastEviction(a)) {
    std::printf("FAIL: the oldest slot (a) was not overwritten by the fifth peer\n");
    ok = false;
  }
  for (uint8_t tag : {uint8_t{0xB2}, uint8_t{0xC3}, uint8_t{0xD4}, uint8_t{0xE5}}) {
    if (!ring.lastEviction(prefix(tag))) {
      std::printf("FAIL: peer %02x forgotten while the ring had room for it\n", tag);
      ok = false;
    }
  }
  if (Ring::capacity() != 4) {
    std::printf("FAIL: capacity %zu\n", Ring::capacity());
    ok = false;
  }

  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
