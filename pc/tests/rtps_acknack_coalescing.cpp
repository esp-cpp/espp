// A reliable reader coalesces its HEARTBEAT answers.
//
// RTPS 2.5 8.4.12.2: a HEARTBEAT the reader misses nothing for needs no
// ACKNACK when the writer set its FinalFlag (Cyclone DDS sets it on the
// HEARTBEAT it piggybacks on every DATA). And while the reader IS missing
// samples it answers at most once per SFR_ACKNACK_MIN_INTERVAL_MS per writer,
// as Fast DDS and Cyclone DDS coalesce their answers. The reader used to
// answer every HEARTBEAT (and every GAP it was behind) at once: one that had
// fallen behind a 20 Hz Cyclone writer spiralled, each stale HEARTBEAT it
// worked through earning an ACKNACK, each ACKNACK a GAP + HEARTBEAT back,
// its backlog growing faster than it drained until nothing else got through
// (the QEMU soak's Cyclone cycles lost their 30 KB samples and service calls).
//
// A standalone StatefulReader whose writer proxy points at a loopback socket
// this test reads. Checks:
//  1. HEARTBEAT [1..0] with FinalFlag, nothing missing: no ACKNACK;
//  2. the same without FinalFlag: an ACKNACK (base 1, no bits) - the writer
//     asked;
//  3. HEARTBEAT [1..5], five missing: an ACKNACK requesting them, and ten more
//     such HEARTBEATs right behind it get none (fails on the old reader: ten
//     more ACKNACKs);
//  4. after the interval, the next HEARTBEAT is answered again;
//  5. a GAP the reader is behind (its case 1 answer) is coalesced the same way.
//
// Exits 0 on success, 1 on failure.

#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

#include "rtps/communication/EsppTransport.hpp"
#include "rtps/config.hpp"
#include "rtps/entities/StatefulReader.hpp"
#include "rtps/messages/MessageTypes.hpp"

namespace {

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

void noop_rx(void *, const uint8_t *, std::size_t, rtps::Ip4Port_t, rtps::Ip4Port_t,
             const rtps::Ip4AddressBytes &) {}

// Drain the socket for up to `wait_ms`; true when a datagram carrying an
// ACKNACK submessage arrived, with its base and bit count.
bool acknack_arrived(int sock, int wait_ms, uint32_t &base, uint32_t &nbits) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
  uint8_t buf[1500];
  while (std::chrono::steady_clock::now() < deadline) {
    const ssize_t n = ::recv(sock, buf, sizeof(buf), 0);
    if (n <= 0) {
      continue; // timeout tick
    }
    if (n < 20 || std::memcmp(buf, "RTPS", 4) != 0) {
      continue;
    }
    std::size_t pos = 20;
    while (pos + 4 <= static_cast<std::size_t>(n)) {
      const uint8_t id = buf[pos];
      uint16_t o2n = 0;
      std::memcpy(&o2n, buf + pos + 2, sizeof(o2n));
      if (id == static_cast<uint8_t>(rtps::SubmessageKind::ACKNACK) && pos + 4 + 20 <= n) {
        std::memcpy(&base, buf + pos + 4 + 12, 4); // readerId, writerId, base.high, base.low
        std::memcpy(&nbits, buf + pos + 4 + 16, 4);
        return true;
      }
      if (o2n == 0) {
        break;
      }
      pos += 4 + o2n;
    }
  }
  return false;
}

bool run_checks() {
  const int interval_ms = rtps::Config::SFR_ACKNACK_MIN_INTERVAL_MS;
  CHECK(interval_ms >= 20 && interval_ms <= 1000, "the interval this test is written for");

  // The "writer": a UDP socket on 127.0.0.1 the reader's proxy points at.
  const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  CHECK(sock >= 0, "socket");
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  CHECK(::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0, "bind");
  socklen_t len = sizeof(addr);
  CHECK(::getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) == 0, "getsockname");
  const uint16_t port = ntohs(addr.sin_port);
  timeval tv{0, 20000};
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  rtps::EsppTransport transport(&noop_rx, nullptr);
  rtps::StatefulReader reader;
  rtps::TopicData attributes{};
  std::strncpy(attributes.topicName, "acknack_coalescing", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  attributes.endpointGuid.prefix.id[0] = 0x11;
  attributes.endpointGuid.entityId.entityKey[2] = 0x04;
  attributes.endpointGuid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY;
  attributes.reliabilityKind = rtps::ReliabilityKind_t::RELIABLE;
  attributes.unicastLocator = rtps::FullLengthLocator::createUDPv4Locator(127, 0, 0, 1, 7701);
  CHECK(reader.init(attributes, transport), "reader init");

  rtps::Guid_t writer_guid{};
  writer_guid.prefix.id[0] = 0x22;
  writer_guid.entityId.entityKey[2] = 0x01;
  writer_guid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  const rtps::LocatorIPv4 writer_locator(
      rtps::FullLengthLocator::createUDPv4Locator(127, 0, 0, 1, port));
  CHECK(reader.addNewMatchedWriter(rtps::WriterProxy{writer_guid, writer_locator, true}),
        "addNewMatchedWriter");

  int32_t hb_count = 0;
  const auto heartbeat = [&](uint32_t first, uint32_t last, bool final_flag) {
    rtps::SubmessageHeartbeat msg{};
    msg.header.submessageId = rtps::SubmessageKind::HEARTBEAT;
    msg.header.flags = rtps::FLAG_LITTLE_ENDIAN | (final_flag ? rtps::FLAG_FINAL : 0);
    msg.readerId = attributes.endpointGuid.entityId;
    msg.writerId = writer_guid.entityId;
    msg.firstSN = rtps::SequenceNumber_t{0, first};
    msg.lastSN = rtps::SequenceNumber_t{0, last};
    msg.count.value = ++hb_count;
    reader.onNewHeartbeat(msg, writer_guid.prefix);
  };
  const auto gap = [&](uint32_t start, uint32_t base) {
    rtps::SubmessageGap msg{};
    msg.header.submessageId = rtps::SubmessageKind::GAP;
    msg.header.flags = rtps::FLAG_LITTLE_ENDIAN;
    msg.readerId = attributes.endpointGuid.entityId;
    msg.writerId = writer_guid.entityId;
    msg.gapStart = rtps::SequenceNumber_t{0, start};
    msg.gapList.base = rtps::SequenceNumber_t{0, base};
    msg.gapList.numBits = 0;
    reader.onNewGapMessage(msg, writer_guid.prefix);
  };
  const auto settle = [&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms + 20));
    uint32_t b = 0, n = 0;
    while (acknack_arrived(sock, 20, b, n)) {
    }
  };
  uint32_t base = 0, nbits = 0;

  // 1. Nothing to have, FinalFlag: no answer.
  heartbeat(1, 0, /*final=*/true);
  CHECK(!acknack_arrived(sock, 150, base, nbits),
        "a final HEARTBEAT the reader misses nothing for gets no ACKNACK");

  // 2. Nothing to have, FinalFlag clear: the writer asked, so it gets an ack.
  heartbeat(1, 0, /*final=*/false);
  CHECK(acknack_arrived(sock, 300, base, nbits), "a non-final HEARTBEAT is acknowledged");
  CHECK(base == 1 && nbits == 0, "... with nothing requested");
  settle();

  // 3. Five missing: one request, then ten HEARTBEATs right behind it get none.
  heartbeat(1, 5, /*final=*/true);
  CHECK(acknack_arrived(sock, 300, base, nbits), "a HEARTBEAT with samples missing is answered");
  CHECK(base == 1 && nbits == 5, "... requesting the five");
  for (int i = 0; i < 10; ++i) {
    heartbeat(1, 5, /*final=*/true);
  }
  CHECK(!acknack_arrived(sock, interval_ms / 2, base, nbits),
        "HEARTBEATs within the interval get no further ACKNACK (coalesced)");

  // 4. The interval over: answered again.
  settle();
  heartbeat(1, 5, /*final=*/false);
  CHECK(acknack_arrived(sock, 300, base, nbits), "after the interval the HEARTBEAT is answered");
  CHECK(base == 1 && nbits == 5, "... requesting the five again");

  // 5. A GAP the reader is behind ([3,4] still expected, 5 gone) is answered
  //    once per interval too: not right after the ACKNACK above, then yes.
  gap(1, 3); // 1 and 2 will never come: expected 3 now (no answer of its own)
  gap(5, 6); // behind the gap's start: case 1 asks for 3 and 4 - coalesced
  CHECK(!acknack_arrived(sock, interval_ms / 2, base, nbits),
        "a GAP's request within the interval is coalesced");
  settle();
  gap(5, 6);
  CHECK(acknack_arrived(sock, 300, base, nbits), "after the interval the GAP's request goes out");
  CHECK(base == 3 && nbits == 2, "... for 3 and 4");

  ::close(sock);
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
