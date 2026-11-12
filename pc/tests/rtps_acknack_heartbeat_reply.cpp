// A reliable writer answers a reader that requests nothing while behind with
// a HEARTBEAT at once.
//
// RTPS 2.5 8.4.15.5: an ACKNACK with an empty sequence-number set is a
// request for a HEARTBEAT. Cyclone DDS sends its pre-emptive ACKNACK as base 1
// / count 0 (not base 0, the only form the writer answered) and repeats it
// every second until a HEARTBEAT arrives. The writer left that to the
// periodic heartbeat, up to a full period later; a change written meanwhile
// for the new reader - the reply to the request it sent right after matching
// - was announced by the FIRST heartbeat it ever processed from us, which a
// VOLATILE reader takes for pre-match history: the reply was skipped and the
// service call timed out (the QEMU interop matrix's service call under loss
// against rmw_cyclonedds_cpp, one run in six).
//
// Checks, with the writer's proxy pointed at a loopback socket this test
// reads: an ACKNACK base 1 / no bits with three changes in the history gets
// a HEARTBEAT [1..3] right away; one acknowledging everything (base 4) gets
// nothing; a base-0 ACKNACK (the form already handled) still gets one.
//
// Exits 0 on success, 1 on failure.

#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "rtps/communication/EsppTransport.hpp"
#include "rtps/entities/StatefulWriter.hpp"
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

// Drain the socket for up to `wait_ms`; true when a datagram carrying a
// HEARTBEAT submessage from `writer` arrived, with its first/last numbers.
bool heartbeat_arrived(int sock, int wait_ms, uint32_t &first, uint32_t &last) {
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
      if (id == static_cast<uint8_t>(rtps::SubmessageKind::HEARTBEAT) && pos + 4 + 28 <= n) {
        rtps::SequenceNumber_t f{}, l{};
        std::memcpy(&f.high, buf + pos + 4 + 8, 4);
        std::memcpy(&f.low, buf + pos + 4 + 12, 4);
        std::memcpy(&l.high, buf + pos + 4 + 16, 4);
        std::memcpy(&l.low, buf + pos + 4 + 20, 4);
        first = f.low;
        last = l.low;
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
  // The "reader": a UDP socket on 127.0.0.1 the writer's proxy points at.
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
  timeval tv{0, 50000};
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  rtps::EsppTransport transport(&noop_rx, nullptr);
  rtps::StatefulWriter writer;
  rtps::TopicData attributes{};
  std::strncpy(attributes.topicName, "acknack_heartbeat", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  attributes.endpointGuid.prefix.id[0] = 0x11;
  attributes.endpointGuid.entityId.entityKey[2] = 0x01;
  attributes.endpointGuid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  attributes.reliabilityKind = rtps::ReliabilityKind_t::RELIABLE;
  attributes.unicastLocator = rtps::FullLengthLocator::createUDPv4Locator(127, 0, 0, 1, 7700);
  CHECK(writer.init(attributes, rtps::TopicKind_t::NO_KEY, transport, /*enforceUnicast=*/true),
        "writer init");

  rtps::Guid_t reader_guid{};
  reader_guid.prefix.id[0] = 0x22;
  reader_guid.entityId.entityKey[2] = 0x04;
  reader_guid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY;
  const rtps::LocatorIPv4 reader_locator(
      rtps::FullLengthLocator::createUDPv4Locator(127, 0, 0, 1, port));
  CHECK(writer.addNewMatchedReader(rtps::ReaderProxy{reader_guid, reader_locator, true}),
        "addNewMatchedReader");

  // Three changes in the history (sent to the socket as a side effect; the
  // drain below discards them).
  const uint8_t payload[8] = {0, 1, 0, 0, 1, 2, 3, 4};
  for (int i = 0; i < 3; ++i) {
    CHECK(writer.newChange(rtps::ChangeKind_t::ALIVE, payload, sizeof(payload)) != nullptr,
          "newChange");
  }
  uint32_t first = 0, last = 0;
  while (heartbeat_arrived(sock, 300, first, last)) {
    // drain the heartbeats the sends piggybacked
  }

  const auto acknack = [&](uint32_t base, int32_t count) {
    rtps::SubmessageAckNack msg{};
    msg.readerId = reader_guid.entityId;
    msg.writerId = attributes.endpointGuid.entityId;
    msg.readerSNState.base = rtps::SequenceNumber_t{0, base};
    msg.readerSNState.numBits = 0;
    msg.count.value = count;
    writer.onNewAckNack(msg, reader_guid.prefix);
  };

  // 1. Cyclone's pre-emptive form: base 1, nothing requested, behind -> HEARTBEAT.
  acknack(1, 0);
  CHECK(heartbeat_arrived(sock, 1000, first, last), "an empty ACKNACK at base 1 gets a HEARTBEAT");
  CHECK(first == 1 && last == 3, "the HEARTBEAT announces [1..3]");
  while (heartbeat_arrived(sock, 200, first, last)) {
  }

  // 2. Everything acknowledged (base = last + 1): no heartbeat for it.
  acknack(4, 1);
  CHECK(!heartbeat_arrived(sock, 700, first, last),
        "an ACKNACK acknowledging everything gets no HEARTBEAT (no ping-pong)");

  // 3. The base-0 form the writer always answered still works.
  acknack(0, 2);
  CHECK(heartbeat_arrived(sock, 1000, first, last), "a base-0 ACKNACK gets a HEARTBEAT");

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
