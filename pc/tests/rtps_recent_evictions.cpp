// The admission rule behind Participant::evictUnusedRemoteParticipant on a
// domain that stays bigger than the remote-participant pool (RecentEvictions):
// a peer evicted lately is held off; otherwise one eviction per period from the
// budget of its kind - former evictees and never-evicted peers each have their
// own. It used to be one eviction per period for anyone: the peers that did
// not fit re-announced every period and took turns displacing each other, and
// a late publisher the device needed lost every round for 90 s in the QEMU
// suite (pool_full_evicts_unused_for_late_joiner). This pins the rule, that
// neither budget starves the other, and the ring (oldest eviction overwritten).

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

  // a first-timer: its own budget, never the former evictees'
  ok = expect(ring.gate(a, t0, never, never, cooldown, holdoff), Gate::Proceed,
              "unknown, nothing spent") &&
       ok;
  ok = expect(ring.gate(a, t0, t0 - 100ms, never, cooldown, holdoff), Gate::Proceed,
              "unknown: an eviction for a FORMER evictee 100 ms ago does not count against it") &&
       ok;
  ok = expect(ring.gate(a, t0, never, t0 - 100ms, cooldown, holdoff), Gate::Cooldown,
              "unknown: an eviction for another FIRST-TIMER 100 ms ago does") &&
       ok;
  ok = expect(ring.gate(a, t0, never, t0 - cooldown, cooldown, holdoff), Gate::Proceed,
              "unknown: the first-timer budget is back after one cooldown") &&
       ok;
  if (ring.lastEviction(a)) {
    std::printf("FAIL: an unknown peer has an eviction time\n");
    ok = false;
  }

  // a evicted at t0: held off for holdoff, then on the former evictees' budget
  ring.note(a, t0);
  if (!ring.lastEviction(a) || *ring.lastEviction(a) != t0) {
    std::printf("FAIL: the eviction time of a is not remembered\n");
    ok = false;
  }
  ok = expect(ring.gate(a, t0 + 1s, never, never, cooldown, holdoff), Gate::HeldOff,
              "a one second after") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff - 1ms, never, never, cooldown, holdoff), Gate::HeldOff,
              "a just before the hold-off ends") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff, t0 + holdoff - 500ms, never, cooldown, holdoff),
              Gate::Cooldown, "a past the hold-off, an eviction for a former evictee 500 ms ago") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff, never, t0 + holdoff - 500ms, cooldown, holdoff),
              Gate::Proceed, "a past the hold-off: a stream of first-timers does not starve it") &&
       ok;
  ok = expect(ring.gate(a, t0 + holdoff, t0 + holdoff - cooldown, never, cooldown, holdoff),
              Gate::Proceed, "a past the hold-off, its budget's cooldown just over") &&
       ok;
  // meanwhile b, never evicted, is served right after an eviction made for a
  // former evictee: the budgets are separate
  ok = expect(ring.gate(b, t0 + 1ms, t0, never, cooldown, holdoff), Gate::Proceed,
              "b (first-timer) right after an eviction for a former evictee") &&
       ok;

  // a second eviction of a refreshes its entry rather than adding one
  ring.note(a, t0 + 30s);
  ok = expect(ring.gate(a, t0 + 31s, never, never, cooldown, holdoff), Gate::HeldOff,
              "a held off again") &&
       ok;
  ring.note(b, t0 + 29s); // noted later but with an OLDER eviction time than a's refresh
  ring.note(c, t0 + 32s);
  ring.note(prefix(0xD4), t0 + 33s);
  // the ring (4 entries) is full: a fifth peer overwrites the OLDEST EVICTION
  // (b at t0+29 s), not the oldest slot (a, refreshed to t0+30 s)
  ring.note(prefix(0xE5), t0 + 34s);
  if (ring.lastEviction(b)) {
    std::printf("FAIL: the oldest eviction (b) was not the one overwritten\n");
    ok = false;
  }
  for (uint8_t tag : {uint8_t{0xA1}, uint8_t{0xC3}, uint8_t{0xD4}, uint8_t{0xE5}}) {
    if (!ring.lastEviction(prefix(tag))) {
      std::printf("FAIL: peer %02x forgotten while a more recent eviction was kept\n", tag);
      ok = false;
    }
  }
  if (Ring::capacity() != 4) {
    std::printf("FAIL: capacity %zu\n", Ring::capacity());
    ok = false;
  }
  // a ring of one entry still works (the Participant derives at least 2; this
  // pins that N=1 is not a special case)
  {
    rtps::RecentEvictions<1> one;
    one.note(a, t0);
    one.note(b, t0 + 1s);
    if (one.lastEviction(a) || !one.lastEviction(b)) {
      std::printf("FAIL: a one-entry ring does not keep the newest eviction\n");
      ok = false;
    }
  }

  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
