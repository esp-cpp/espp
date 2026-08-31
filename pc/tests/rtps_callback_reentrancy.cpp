// Reentrant cross-callback removal: while a reader dispatch is invoking its
// callback snapshot, callback A may legally call removeCallback(B) and then
// free B's argument - the removal-completion guarantee must hold even though
// the drain excludes A's own dispatch (the caller). The dispatch loop must
// therefore revalidate each registration against the LIVE table immediately
// before invoking it, or it calls the stale snapshot entry with a freed arg.
//
// Engine-level test (the facade registers a single callback per reader, so
// A-removes-B is only reachable through the engine API). Exits 0 on success.

#include <cstdio>
#include <cstring>

#include "rtps/entities/StatelessReader.hpp"

namespace {
struct TestState {
  rtps::StatelessReader *reader{nullptr};
  rtps::Reader::callbackIdentifier_t b_id{0};
  bool b_arg_freed{false}; // set by A after removing B (simulates freeing)
  bool b_ran_after_free{false};
  bool a_ran{false};
  bool b_ran{false};
};

void callback_a(void *arg, const rtps::ReaderCacheChange &) {
  auto *st = static_cast<TestState *>(arg);
  st->a_ran = true;
  // A removes B and "frees" B's argument - the documented removal-completion
  // guarantee says B must not run after this returns.
  st->reader->removeCallback(st->b_id);
  st->b_arg_freed = true;
}

void callback_b(void *arg, const rtps::ReaderCacheChange &) {
  auto *st = static_cast<TestState *>(arg);
  st->b_ran = true;
  if (st->b_arg_freed) {
    st->b_ran_after_free = true; // use-after-free in a real application
  }
}
} // namespace

int main() {
  rtps::StatelessReader reader;
  rtps::TopicData attributes{};
  std::strncpy(attributes.topicName, "reentrancy", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  if (!reader.init(attributes)) {
    std::printf("FAIL: reader init\n");
    return 1;
  }

  TestState st;
  st.reader = &reader;
  // Registration order matters: A must be invoked BEFORE B in the dispatch
  // loop so its removal targets a not-yet-invoked snapshot entry.
  const auto a_id = reader.registerCallback(&callback_a, &st);
  st.b_id = reader.registerCallback(&callback_b, &st);
  if (a_id == 0 || st.b_id == 0) {
    std::printf("FAIL: registerCallback\n");
    return 1;
  }

  const uint8_t payload[4] = {1, 2, 3, 4};
  rtps::Guid_t writer_guid{};
  rtps::ReaderCacheChange change{rtps::ChangeKind_t::ALIVE, writer_guid,
                                 rtps::SequenceNumber_t{0, 1}, payload, sizeof(payload)};
  reader.newChangeIfCurrent(reader.generation(), change);

  if (!st.a_ran) {
    std::printf("FAIL: callback A never ran\n");
    return 1;
  }
  if (st.b_ran_after_free) {
    std::printf("FAIL: callback B invoked AFTER removeCallback(B) returned and its arg was "
                "freed (stale snapshot entry)\n");
    return 1;
  }
  if (st.b_ran) {
    // B ran before its removal completed - impossible here (A precedes B and
    // removes it synchronously), so treat as a harness error.
    std::printf("FAIL: unexpected ordering (B ran before A's removal)\n");
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
