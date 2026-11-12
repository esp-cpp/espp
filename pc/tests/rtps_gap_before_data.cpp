// A GAP is applied only for sequence numbers the writer has announced.
//
// Fast DDS, after an endpoint deletion, sends its SEDP readers a GAP for the
// NEXT sequence number of its publications writer (a hole where the deleted
// endpoint's announcement stood), and then writes that very number for the
// next endpoint it announces: GAP [13,14) followed by DATA 13. The reliable
// reader used to apply the GAP as "13 will never come", skip to 14, and then
// drop DATA 13 as stale - and every announcement after it, since the writer
// repeats the pattern for each deletion. A peer that churned its publishers
// (the interop matrix's churn case against rmw_fastrtps_cpp) was matched once
// and never again.
//
// The rule now: a GAP wholly above the highest number the writer has shown to
// exist (the lastSN of its HEARTBEATs, or a DATA) is ignored, and one reaching
// past it is applied only up to it. The next HEARTBEAT settles what exists.
//
// Checks, feeding a reliable reader's proxy directly:
//  1. HEARTBEAT [1..12], then GAP [13,14): expected stays 13, and DATA 13 is
//     delivered (fails without the fix: 13 was skipped).
//  2. A GAP within the announced range still applies: HEARTBEAT [1..15],
//     GAP [14,15) with 14 expected -> 15 expected, DATA 15 delivered.
//  3. A GAP reaching past the announced range is applied only up to it:
//     HEARTBEAT [1..17], GAP [16,20) -> expected 18, DATA 18 delivered.
//  4. A DATA counts as an announcement too: DATA 20 out of order (dropped),
//     then GAP [19,20) applies -> expected 20, DATA 20 delivered.
//
// Exits 0 on success, 1 on the first failed check.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/messages/MessageTypes.hpp"

namespace {

constexpr rtps::Ip4AddressBytes kIp{127, 0, 0, 1};

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

struct Delivered {
  std::vector<uint32_t> sns;
};

void on_change(void *arg, const rtps::ReaderCacheChange &change) {
  static_cast<Delivered *>(arg)->sns.push_back(change.sn.low);
}

constexpr uint8_t kPayload[4] = {1, 2, 3, 4};

void feed_data(rtps::Reader *reader, const rtps::Guid_t &writer, uint32_t sn) {
  rtps::Guid_t guid = writer;
  rtps::ReaderCacheChange change{rtps::ChangeKind_t::ALIVE, guid, rtps::SequenceNumber_t{0, sn},
                                 kPayload, sizeof(kPayload)};
  reader->newChangeIfCurrent(reader->generation(), change);
}

void feed_heartbeat(rtps::Reader *reader, const rtps::Guid_t &writer, uint32_t first, uint32_t last,
                    int32_t count) {
  rtps::SubmessageHeartbeat hb{};
  hb.readerId = reader->m_attributes.endpointGuid.entityId;
  hb.writerId = writer.entityId;
  hb.firstSN = rtps::SequenceNumber_t{0, first};
  hb.lastSN = rtps::SequenceNumber_t{0, last};
  hb.count.value = count;
  reader->onNewHeartbeatIfCurrent(reader->generation(), hb, writer.prefix);
}

// GAP [start, base): the numbers from start up to (not including) base, as
// Fast DDS sends them (numBits 0).
void feed_gap(rtps::Reader *reader, const rtps::Guid_t &writer, uint32_t start, uint32_t base) {
  rtps::SubmessageGap gap{};
  gap.readerId = reader->m_attributes.endpointGuid.entityId;
  gap.writerId = writer.entityId;
  gap.gapStart = rtps::SequenceNumber_t{0, start};
  gap.gapList.base = rtps::SequenceNumber_t{0, base};
  gap.gapList.numBits = 0;
  reader->onNewGapIfCurrent(reader->generation(), gap, writer.prefix);
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

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");
  rtps::Reader *reader =
      domain.createReader(*part, "gap_before_data", "GapType", /*reliable=*/true, {0, 0, 0, 0});
  CHECK(reader != nullptr, "createReader (reliable)");
  Delivered delivered;
  CHECK(reader->registerCallback(&on_change, &delivered) != 0, "registerCallback");

  rtps::Guid_t writer{};
  writer.prefix.id[0] = 0x01;
  writer.prefix.id[11] = 0x42;
  writer.entityId.entityKey[2] = 0x01;
  writer.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  CHECK(reader->addNewMatchedWriter(rtps::WriterProxy{writer, rtps::LocatorIPv4{}, true}),
        "addNewMatchedWriter");
  rtps::WriterProxy *proxy = reader->getProxy(writer);
  CHECK(proxy != nullptr, "getProxy");

  // Bring the reader to "expecting 13": the first 12 delivered in order.
  for (uint32_t sn = 1; sn <= 12; ++sn) {
    feed_data(reader, writer, sn);
  }
  CHECK(delivered.sns.size() == 12 && proxy->expectedSN.low == 13, "1..12 delivered, expecting 13");
  delivered.sns.clear();

  // 1. The Fast DDS pattern: HEARTBEAT [1..12], GAP [13,14), DATA 13.
  feed_heartbeat(reader, writer, 1, 12, 1);
  feed_gap(reader, writer, 13, 14);
  CHECK(proxy->expectedSN.low == 13, "a GAP beyond the announced range does not advance");
  feed_data(reader, writer, 13);
  CHECK(same(delivered.sns, {13}, "DATA 13 after the premature GAP"), "DATA 13 delivered");
  delivered.sns.clear();

  // 2. A GAP inside the announced range applies.
  feed_heartbeat(reader, writer, 1, 15, 2);
  feed_gap(reader, writer, 14, 15);
  CHECK(proxy->expectedSN.low == 15, "a GAP within the announced range advances past the hole");
  feed_data(reader, writer, 15);
  CHECK(same(delivered.sns, {15}, "DATA 15 after the applied GAP"), "DATA 15 delivered");
  delivered.sns.clear();

  // 3. A GAP reaching past the announced range applies up to it only.
  feed_heartbeat(reader, writer, 1, 17, 3);
  feed_gap(reader, writer, 16, 20);
  CHECK(proxy->expectedSN.low == 18, "a GAP past the announced range stops at announced + 1");
  feed_data(reader, writer, 18);
  CHECK(same(delivered.sns, {18}, "DATA 18 after the capped GAP"), "DATA 18 delivered");
  delivered.sns.clear();

  // 4. A DATA announces its number too: 20 arrives early (dropped, 19 is
  //    expected), then GAP [19,20) is within what the writer has shown.
  feed_data(reader, writer, 20);
  CHECK(delivered.sns.empty() && proxy->expectedSN.low == 19, "DATA 20 early: dropped");
  feed_gap(reader, writer, 19, 20);
  CHECK(proxy->expectedSN.low == 20, "a GAP up to a number a DATA announced applies");
  feed_data(reader, writer, 20);
  CHECK(same(delivered.sns, {20}, "DATA 20 after the GAP"), "DATA 20 delivered");

  return true;
}

} // namespace

int main() {
  if (!run_checks()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
