// A participant's SEDP writers never lose a live endpoint announcement.
//
// The two SEDP writers keep one change per user endpoint of the participant,
// and a KEEP_LAST ring that drops its oldest change when full (the static,
// embedded storage) would silently un-discover the oldest endpoint for every
// peer joining later - and, its change gone, that endpoint could then never be
// deleted (SEDPAgent::deleteWriter failed, the facade re-attached it, the slot
// leaked). The embedded profile's defaults put HISTORY_SIZE_STATEFUL (10) at
// exactly the per-participant endpoint cap (10, of which 3 builtin), so a few
// deletions within the dispose retention window were enough to hit it.
//
// Three parts:
//  1. The ring policy, on a static ring compiled into this test alone
//     (RTPS_STORAGE_STATIC; the library is built with dynamic storage, and the
//     ring is a header-only template this translation unit instantiates at a
//     size the library never does): when full, a sent dispose-after-write
//     change is given up first (it only waits for a retransmission), an unsent
//     one is not, and with none to give up the ring says it would drop.
//  2. An endpoint whose announcement is no longer in the SEDP history (as an
//     older engine's overwrite left it) can still be deleted, and its slot is
//     reusable: the participant's writer slots are filled, the first writer's
//     SEDP change is removed behind the agent's back, the writer is deleted,
//     and one more writer can be created.
//  3. The caps agree: with static storage the history covers the user
//     endpoints of a participant (config.hpp's static_assert, restated here
//     with the same arithmetic so a profile change shows up in this test).
//
// Exits 0 on success, 1 on the first failed check.

#define RTPS_STORAGE_STATIC
#include "rtps/storages/HistoryCacheWithDeletion.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/entities/StatefulWriter.hpp"
#include "rtps/entities/Writer.hpp"

namespace {

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

constexpr rtps::Ip4AddressBytes kIp{127, 0, 0, 1};

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

bool delete_without_sedp_change() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");

  // Fill the participant's writer slots (3 are the builtin discovery writers).
  const int user_writers = static_cast<int>(rtps::Config::NUM_WRITERS_PER_PARTICIPANT) - 3;
  std::vector<rtps::Writer *> writers;
  for (int i = 0; i < user_writers; ++i) {
    const std::string topic = "overflow_" + std::to_string(i);
    rtps::Writer *w = domain.createWriter(*part, topic.c_str(), "OverflowType",
                                          /*reliable=*/true, /*enforceUnicast=*/false);
    CHECK(w != nullptr, "createWriter up to the per-participant cap");
    writers.push_back(w);
  }
  CHECK(domain.createWriter(*part, "one_too_many", "OverflowType", true, false) == nullptr,
        "the cap holds");

  // Its SEDP announcement disappears behind the agent's back (what an
  // overwrite by an older engine looked like).
  auto *sedp = static_cast<rtps::StatefulWriter *>(
      part->getWriter(rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER));
  CHECK(sedp != nullptr, "SEDP publications writer");
  CHECK(sedp->removeFromHistory(writers.front()->getSEDPSequenceNumber()),
        "the first writer's announcement removed from the SEDP history");
  CHECK(!sedp->removeFromHistory(writers.front()->getSEDPSequenceNumber()), "and it is gone");

  // The writer can still be deleted, and its slot is free again.
  CHECK(domain.deleteWriter(*part, writers.front()),
        "deleting a writer whose SEDP change is gone succeeds");
  rtps::Writer *again = domain.createWriter(*part, "overflow_again", "OverflowType", true, false);
  CHECK(again != nullptr, "the slot of the deleted writer is reusable");
  CHECK(domain.deleteWriter(*part, again), "and that writer deletes normally");
  return true;
}

bool caps_agree() {
  // The same arithmetic as config.hpp's static_assert (3 builtin writers and
  // 3 builtin readers per participant are not announced).
  CHECK(rtps::Config::HISTORY_SIZE_STATEFUL + 3 >= rtps::Config::NUM_WRITERS_PER_PARTICIPANT,
        "HISTORY_SIZE_STATEFUL covers the user writers of a participant");
  CHECK(rtps::Config::HISTORY_SIZE_STATEFUL + 3 >= rtps::Config::NUM_READERS_PER_PARTICIPANT,
        "HISTORY_SIZE_STATEFUL covers the user readers of a participant");
  return true;
}

} // namespace

int main() {
  if (!ring_policy() || !delete_without_sedp_change() || !caps_agree()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
