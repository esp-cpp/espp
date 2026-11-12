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
// The ring's own policy (give up a sent dispose, else refuse) is pinned on a
// static ring in pc/tests/rtps_sedp_history_ring.cpp, a translation unit that
// links no engine object; this test covers the engine side with the host
// library as built:
//  1. An endpoint whose announcement is no longer in the SEDP history (as an
//     older engine's overwrite left it) can still be deleted, and its slot is
//     reusable: the participant's writer slots are filled, the first writer's
//     SEDP change is removed behind the agent's back, the writer is deleted,
//     and one more writer can be created.
//  2. A creation the participant refuses gives the pooled endpoint back: with
//     the participant's slots full, the refused writers do not exhaust the
//     global stateful pool, so once a slot frees up a writer can be created.
//  3. The caps agree: with static storage the history covers the user
//     endpoints of a participant (config.hpp's static_assert, restated here
//     with the same arithmetic so a profile change shows up in this test).
//
// Exits 0 on success, 1 on the first failed check.

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
  // 2. Refused creations must not consume the global pool: more refusals than
  //    the pool has slots, then a slot frees up and a writer still fits.
  for (int i = 0; i < static_cast<int>(rtps::Config::NUM_STATEFUL_WRITERS) + 2; ++i) {
    CHECK(domain.createWriter(*part, "refused", "OverflowType", true, false) == nullptr,
          "still refused while the participant is full");
  }
  CHECK(domain.deleteWriter(*part, writers.back()), "free one slot");
  rtps::Writer *fits = domain.createWriter(*part, "fits_again", "OverflowType", true, false);
  CHECK(fits != nullptr, "refused creations did not exhaust the stateful writer pool");
  writers.back() = fits;

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
  if (!delete_without_sedp_change() || !caps_agree()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
