// Best-effort delivery still rejects what arrives out of order (RTPS 2.5
// 8.4.12.1): a StatelessReader must deliver a change from a matched writer
// only when its sequence number is above the highest already received from
// that writer, so a duplicated or reordered datagram never reaches the
// application twice, or late. Found by the QEMU interop matrix's duplication
// and reordering cases (components/rtps/interop/qemu); this is the host-side
// regression test for the rule, feeding the reader directly.
//
// Also pinned: a change from a writer the reader has no proxy for is still
// delivered (there is nothing to track it against), and writers are tracked
// independently.

#include <cstdio>
#include <cstring>
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

bool feed(rtps::StatelessReader &reader, const rtps::Guid_t &writer, uint32_t sn) {
  const uint8_t payload[4] = {1, 2, 3, 4};
  rtps::Guid_t guid = writer; // the change keeps a non-const reference
  rtps::ReaderCacheChange change{rtps::ChangeKind_t::ALIVE, guid, rtps::SequenceNumber_t{0, sn},
                                 payload, sizeof(payload)};
  reader.newChangeIfCurrent(reader.generation(), change);
  return true;
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

  // no proxy for the writer: delivered as before, every time
  delivered.sns.clear();
  feed(reader, unknown, 7);
  feed(reader, unknown, 7);
  feed(reader, unknown, 6);
  ok = same(delivered.sns, {7, 7, 6}, "unknown writer") && ok;

  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
