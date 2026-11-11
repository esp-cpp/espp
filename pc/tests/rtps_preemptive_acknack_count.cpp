// A reader's pre-emptive ACKNACK (sent when it matches a reliable writer, see
// SEDPAgent) is numbered from the writer proxy's own counter, so the regular
// ACKNACKs that follow carry strictly higher counts. A Fast DDS writer discards
// an ACKNACK whose count is not above the last one it saw from that reader
// (ReaderProxy::check_and_set_acknack_count); with the pre-emptive one fixed at
// 1 and the counter also starting at 1, the reader's first repair request was a
// discarded duplicate, and a fragmented sample from a publisher that was gone
// before the next heartbeat round never arrived (the QEMU soak's 30 KB sample).
#include <cstdio>
#include <cstring>

#include "rtps/communication/EsppTransport.hpp"
#include "rtps/entities/StatefulReader.hpp"

namespace {
int failures = 0;
void check(bool ok, const char *what) {
  std::printf("%s: %s\n", ok ? "ok" : "FAIL", what);
  if (!ok) {
    ++failures;
  }
}
void noop_rx(void *, const uint8_t *, std::size_t, rtps::Ip4Port_t, rtps::Ip4Port_t,
             const rtps::Ip4AddressBytes &) {}
rtps::Guid_t writer_guid(uint8_t id) {
  rtps::Guid_t guid{};
  guid.prefix.id[0] = id;
  guid.entityId.entityKey[0] = id;
  guid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  return guid;
}
} // namespace

int main() {
  rtps::EsppTransport transport(&noop_rx, nullptr); // outlives the reader
  rtps::StatefulReader reader;
  rtps::TopicData attributes{};
  std::strncpy(attributes.topicName, "preemptive_acknack_count", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  if (!reader.init(attributes, transport)) {
    std::printf("FAIL: reader init\n");
    return 1;
  }
  const rtps::Guid_t stored_guid = writer_guid(1);
  const rtps::Guid_t other_guid = writer_guid(2);
  const rtps::WriterProxy stored{stored_guid, rtps::LocatorIPv4{}, /*reliable=*/true};
  const rtps::WriterProxy other{other_guid, rtps::LocatorIPv4{}, /*reliable=*/true};
  if (!reader.addNewMatchedWriter(stored) || !reader.addNewMatchedWriter(other)) {
    std::printf("FAIL: addNewMatchedWriter\n");
    return 1;
  }

  // The SEDP agent's call on match: the pre-emptive ACKNACK takes count 1 from
  // the stored proxy, so the first regular ACKNACK to that writer is 2.
  reader.sendPreemptiveAckNack(stored);
  rtps::WriterProxy *p = reader.getProxy(stored_guid);
  check(p != nullptr, "the matched writer has a proxy");
  if (p == nullptr) {
    return 1;
  }
  check(p->getNextAckNackCount().value == 2,
        "the first regular ACKNACK after the pre-emptive one is numbered 2");

  // Independent per writer: the other proxy's counter is untouched.
  rtps::WriterProxy *q = reader.getProxy(other_guid);
  check(q != nullptr && q->getNextAckNackCount().value == 1,
        "an unrelated writer's first ACKNACK is still numbered 1");

  // Every pre-emptive ACKNACK consumes a count too, so a re-match keeps the
  // sequence strictly increasing: 1 (pre-emptive), 2 (regular), 3 (pre-emptive)
  // above, hence 4 next.
  reader.sendPreemptiveAckNack(stored);
  check(p->getNextAckNackCount().value == 4,
        "a second pre-emptive ACKNACK is numbered after the regular ones");

  // A proxy that is not stored falls back to count 1 and must not crash.
  reader.sendPreemptiveAckNack(rtps::WriterProxy{writer_guid(3), rtps::LocatorIPv4{}, true});
  check(reader.getProxy(writer_guid(3)) == nullptr, "a stray proxy is not stored by the send");

  if (failures != 0) {
    std::printf("FAIL (%d)\n", failures);
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
