// The writer history ring's policy for a builtin (SEDP) writer, on STATIC
// storage: when full, a sent dispose-after-write change is given up first (it
// only waits for a retransmission), an unsent one is not, and with none to
// give up the ring says it would drop - which StatefulWriter::newChange turns
// into a refused announcement for a builtin writer (see
// pc/tests/rtps_sedp_history_overflow.cpp for the engine-level half).
//
// RTPS_STORAGE_STATIC is defined for THIS translation unit only, and the unit
// includes nothing but the header-only ring template, instantiated at a size
// the library never uses: no engine object (Domain, Participant, ...) whose
// layout the storage policy changes is constructed or linked here, so the
// library's dynamic-storage build and this static ring never meet.
//
// Exits 0 on success, 1 on the first failed check.

#define RTPS_STORAGE_STATIC
#include "rtps/storages/HistoryCacheWithDeletion.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>

namespace {

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

bool ring_policy() {
  rtps::HistoryCacheWithDeletion<4> ring;
  const uint8_t payload[4] = {1, 2, 3, 4};
  const auto add = [&](bool dispose_after_write) {
    return ring.addChange(payload, sizeof(payload), /*inLineQoS=*/dispose_after_write,
                          dispose_after_write);
  };
  const auto has = [&](uint32_t sn) {
    return ring.getChangeBySN(rtps::SequenceNumber_t{0, sn}) != nullptr;
  };

  CHECK(!ring.wouldDropOldestOnAdd(), "an empty ring would not drop");
  CHECK(add(false) != nullptr, "SN 1 added");
  CHECK(add(true) != nullptr, "SN 2 added (dispose-after-write, unsent)");
  CHECK(add(true) != nullptr, "SN 3 added (dispose-after-write, to be sent)");
  CHECK(add(false) != nullptr, "SN 4 added");
  CHECK(ring.isFull(), "four changes fill a ring of four");
  CHECK(ring.wouldDropOldestOnAdd(), "a full static ring would drop its oldest on add");

  // Nothing sent yet: an unsent dispose still has to go out, so none is given up.
  CHECK(!ring.dropOldestSentDisposeAfterWrite(), "no sent dispose-after-write to give up");
  CHECK(has(1) && has(2) && has(3) && has(4), "nothing dropped");

  // SN 3 was sent (the writer stamps sentTime on send); SN 2 was not. The
  // sent one goes, the unsent one and the live changes stay.
  ring.getChangeBySN(rtps::SequenceNumber_t{0, 3})->sentTime = std::chrono::steady_clock::now();
  CHECK(ring.dropOldestSentDisposeAfterWrite(), "the sent dispose-after-write is given up");
  CHECK(has(1) && has(2) && !has(3) && has(4), "only SN 3 dropped");
  CHECK(!ring.wouldDropOldestOnAdd(), "room again");
  CHECK(add(false) != nullptr, "SN 5 added into the freed slot");
  CHECK(has(1) && has(2) && has(4) && has(5), "SN 1, 2, 4 and 5 present");
  CHECK(ring.wouldDropOldestOnAdd(), "full again");
  CHECK(!ring.dropOldestSentDisposeAfterWrite(), "nothing left to give up: the writer refuses");
  return true;
}

} // namespace

int main() {
  if (!ring_policy()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
