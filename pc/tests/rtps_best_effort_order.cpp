// Best-effort delivery still rejects what arrives out of order (RTPS 2.5
// 8.4.12.1): a StatelessReader must deliver a change from a matched writer
// only when its sequence number is above the highest already received from
// that writer, so a duplicated or reordered datagram never reaches the
// application twice, or late. Found by the QEMU interop matrix's duplication
// and reordering cases (components/rtps/interop/qemu); this is the host-side
// regression test for the rule, feeding the reader directly.
//
// Also pinned: a change from a writer a USER reader has no proxy for is
// dropped (there is nothing to apply the rule to, and Fast DDS / Cyclone DDS
// drop it too), the builtin SPDP reader still takes announcements from the
// unmatched writers that send them, and writers are tracked independently.

#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

#include "rtps/entities/StatelessReader.hpp"

namespace {
struct Delivered {
  std::vector<uint32_t> sns;
};

void on_change(void *arg, const rtps::ReaderCacheChange &change) {
  static_cast<Delivered *>(arg)->sns.push_back(change.sn.low);
}

rtps::Guid_t writer_guid(uint8_t id) {
  rtps::Guid_t guid{};
  guid.prefix.id[0] = id;
  guid.entityId.entityKey[0] = id;
  guid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  return guid;
}

// The change does not own its bytes and the reader delivers synchronously, so
// a local payload would do; static storage keeps the test independent of that.
constexpr uint8_t kPayload[4] = {1, 2, 3, 4};

void feed(rtps::StatelessReader &reader, const rtps::Guid_t &writer, uint32_t sn) {
  rtps::Guid_t guid = writer; // the change keeps a non-const reference
  rtps::ReaderCacheChange change{rtps::ChangeKind_t::ALIVE, guid, rtps::SequenceNumber_t{0, sn},
                                 kPayload, sizeof(kPayload)};
  reader.newChangeIfCurrent(reader.generation(), change);
}

bool same(const std::vector<uint32_t> &got, const std::vector<uint32_t> &want, const char *what) {
  if (got == want) {
    return true;
  }
  std::printf("FAIL: %s: delivered", what);
  for (auto sn : got) {
    std::printf(" %u", sn);
  }
  std::printf(", expected");
  for (auto sn : want) {
    std::printf(" %u", sn);
  }
  std::printf("\n");
  return false;
}
} // namespace

int main() {
  rtps::StatelessReader reader;
  rtps::TopicData attributes{};
  std::strncpy(attributes.topicName, "best_effort_order", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  // a USER reader (the Participant gives real ones this kind): the unmatched
  // rule below depends on it
  attributes.endpointGuid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY;
  if (!reader.init(attributes)) {
    std::printf("FAIL: reader init\n");
    return 1;
  }
  Delivered delivered;
  if (reader.registerCallback(&on_change, &delivered) == 0) {
    std::printf("FAIL: registerCallback\n");
    return 1;
  }

  const rtps::Guid_t matched = writer_guid(1);
  const rtps::Guid_t other = writer_guid(2);
  const rtps::Guid_t unknown = writer_guid(3);
  if (!reader.addNewMatchedWriter(rtps::WriterProxy{matched, rtps::LocatorIPv4{}, false}) ||
      !reader.addNewMatchedWriter(rtps::WriterProxy{other, rtps::LocatorIPv4{}, false})) {
    std::printf("FAIL: addNewMatchedWriter\n");
    return 1;
  }

  bool ok = true;

  // in order: every one delivered
  feed(reader, matched, 1);
  feed(reader, matched, 2);
  ok = same(delivered.sns, {1, 2}, "in order") && ok;

  // a duplicate of the last one: dropped
  delivered.sns.clear();
  feed(reader, matched, 2);
  ok = same(delivered.sns, {}, "duplicate of the latest") && ok;

  // reordered: 4 arrives before 3 - 4 is delivered, 3 (now late) is dropped, 5 follows
  delivered.sns.clear();
  feed(reader, matched, 4);
  feed(reader, matched, 3);
  feed(reader, matched, 5);
  ok = same(delivered.sns, {4, 5}, "reordered") && ok;

  // a duplicate from further back: dropped
  delivered.sns.clear();
  feed(reader, matched, 1);
  ok = same(delivered.sns, {}, "old duplicate") && ok;

  // a gap is jumped, nothing is waited for (best effort never requests a resend)
  delivered.sns.clear();
  feed(reader, matched, 50);
  feed(reader, matched, 6);
  feed(reader, matched, 51);
  ok = same(delivered.sns, {50, 51}, "gap") && ok;

  // writers are tracked independently: the other writer starts at 1 again
  delivered.sns.clear();
  feed(reader, other, 1);
  feed(reader, other, 1);
  feed(reader, other, 2);
  ok = same(delivered.sns, {1, 2}, "second writer") && ok;

  // no proxy for the writer: a user reader drops it (the ordering rule has
  // nothing to work with, and the matched stream that follows stays ordered)
  delivered.sns.clear();
  feed(reader, unknown, 7);
  feed(reader, unknown, 7);
  feed(reader, unknown, 6);
  ok = same(delivered.sns, {}, "unknown writer on a user reader") && ok;

  // ... and delivery starts once it is matched, from the first sample that
  // arrives after the match (what came before the match stays lost)
  delivered.sns.clear();
  if (!reader.addNewMatchedWriter(rtps::WriterProxy{unknown, rtps::LocatorIPv4{}, false})) {
    std::printf("FAIL: addNewMatchedWriter (late)\n");
    return 1;
  }
  feed(reader, unknown, 8);
  feed(reader, unknown, 7);
  feed(reader, unknown, 9);
  ok = same(delivered.sns, {8, 9}, "late-matched writer") && ok;

  // the builtin SPDP reader takes announcements from writers it has not
  // matched (it never matches them: discovery starts there), unordered
  {
    rtps::StatelessReader spdp;
    rtps::TopicData spdp_attributes{};
    std::strncpy(spdp_attributes.topicName, "DCPSParticipant",
                 sizeof(spdp_attributes.topicName) - 1);
    std::strncpy(spdp_attributes.typeName, "ParticipantProxy",
                 sizeof(spdp_attributes.typeName) - 1);
    spdp_attributes.endpointGuid.entityId = rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER;
    Delivered announcements;
    if (!spdp.init(spdp_attributes) || spdp.registerCallback(&on_change, &announcements) == 0) {
      std::printf("FAIL: spdp reader init\n");
      return 1;
    }
    feed(spdp, unknown, 7);
    feed(spdp, unknown, 7);
    feed(spdp, unknown, 6);
    ok = same(announcements.sns, {7, 7, 6}, "unknown writer on the SPDP reader") && ok;
  }

  // the top of the sequence-number range came off the wire: delivered, no
  // overflow of the next-expected; what is below it afterwards is dropped
  delivered.sns.clear();
  {
    const uint8_t payload[1] = {0};
    rtps::Guid_t guid = matched;
    rtps::ReaderCacheChange top{rtps::ChangeKind_t::ALIVE, guid,
                                rtps::SequenceNumber_t{std::numeric_limits<int32_t>::max(),
                                                       std::numeric_limits<uint32_t>::max()},
                                payload, sizeof(payload)};
    reader.newChangeIfCurrent(reader.generation(), top);
  }
  feed(reader, matched, 60);
  ok = same(delivered.sns, {std::numeric_limits<uint32_t>::max()}, "top of the range") && ok;

  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
