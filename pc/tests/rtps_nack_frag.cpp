// A reliable reader asks for the missing fragments of a partly received
// sample by NACK_FRAG.
//
// RTPS 2.5 8.3.7.10 / 8.4.12.2.5: a reader that holds some fragments of a
// sample names the ones it lacks in a NACK_FRAG. Cyclone DDS answers an
// ACKNACK that names a fragmented sample with its FIRST fragment only and
// leaves the rest to NACK_FRAG; this reader never sent one, so a 30 KB sample
// a Cyclone publisher wrote just before the device's reader had matched it
// (the writer resent fragment 1 on every ACKNACK, 20 times a cycle) never
// completed, and held the single reassembly slot against every sample after
// it: the QEMU soak's Cyclone cycles delivered none of their 30 KB samples.
//
// A standalone StatefulReader whose writer proxy points at a loopback socket
// this test reads. A 30 000-byte sample in 1400-byte fragments (22). Checks:
//  1. fragment 1 held, HEARTBEAT [1..1]: the answer carries an ACKNACK for 1
//     and a NACK_FRAG for sample 1 from fragment 2, 21 bits, all set (fails on
//     the old reader: no NACK_FRAG);
//  2. fragments 2..22 fed: the sample is delivered;
//  3. nothing under reassembly: the next answer has no NACK_FRAG;
//  4. fragments 1 and 5 of sample 2 held: the NACK_FRAG starts at 2 with
//     the bit for 5 clear;
//  5. a HEARTBEAT that does not announce the held sample gets no NACK_FRAG.
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
#include <vector>

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

struct Delivered {
  int count = 0;
  std::size_t size = 0;
};
void on_change(void *arg, const rtps::ReaderCacheChange &change) {
  auto *d = static_cast<Delivered *>(arg);
  ++d->count;
  d->size = change.getDataSize();
}

// What one answer datagram carried.
struct Answer {
  bool acknack = false;
  bool nackfrag = false;
  uint32_t sn = 0;    // NACK_FRAG writerSN.low
  uint32_t base = 0;  // NACK_FRAG fragment set base
  uint32_t nbits = 0; // ... and bit count
  uint32_t word0 = 0; // ... and its first bitmap word (host order)
};

// Drain the socket for up to `wait_ms`; true when an RTPS datagram with an
// ACKNACK arrived, describing it.
bool answer_arrived(int sock, int wait_ms, Answer &a) {
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
    a = Answer{};
    std::size_t pos = 20;
    while (pos + 4 <= static_cast<std::size_t>(n)) {
      const uint8_t id = buf[pos];
      uint16_t o2n = 0;
      std::memcpy(&o2n, buf + pos + 2, sizeof(o2n));
      const std::size_t body = pos + 4;
      if (id == static_cast<uint8_t>(rtps::SubmessageKind::ACKNACK)) {
        a.acknack = true;
      } else if (id == static_cast<uint8_t>(rtps::SubmessageKind::NACK_FRAG) && body + 24 <= n) {
        a.nackfrag = true;
        std::memcpy(&a.sn, buf + body + 12, 4); // readerId, writerId, writerSN.high, .low
        std::memcpy(&a.base, buf + body + 16, 4);
        std::memcpy(&a.nbits, buf + body + 20, 4);
        if (a.nbits != 0 && body + 28 <= n) {
          std::memcpy(&a.word0, buf + body + 24, 4);
        }
      }
      if (o2n == 0) {
        break;
      }
      pos = body + o2n;
    }
    if (a.acknack) {
      return true;
    }
  }
  return false;
}

constexpr uint32_t kSampleSize = 30000;
constexpr uint16_t kFragmentSize = 1400;
constexpr uint32_t kFragments = (kSampleSize + kFragmentSize - 1) / kFragmentSize; // 22

bool run_checks() {
  const int interval_ms = rtps::Config::SFR_ACKNACK_MIN_INTERVAL_MS;

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
  std::strncpy(attributes.topicName, "nack_frag", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  attributes.endpointGuid.prefix.id[0] = 0x11;
  attributes.endpointGuid.entityId.entityKey[2] = 0x04;
  attributes.endpointGuid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY;
  attributes.reliabilityKind = rtps::ReliabilityKind_t::RELIABLE;
  attributes.unicastLocator = rtps::FullLengthLocator::createUDPv4Locator(127, 0, 0, 1, 7702);
  CHECK(reader.init(attributes, transport), "reader init");
  Delivered delivered;
  reader.registerCallback(&on_change, &delivered);

  rtps::Guid_t writer_guid{};
  writer_guid.prefix.id[0] = 0x22;
  writer_guid.entityId.entityKey[2] = 0x01;
  writer_guid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  const rtps::LocatorIPv4 writer_locator(
      rtps::FullLengthLocator::createUDPv4Locator(127, 0, 0, 1, port));
  CHECK(reader.addNewMatchedWriter(rtps::WriterProxy{writer_guid, writer_locator, true}),
        "addNewMatchedWriter");

  std::vector<uint8_t> sample(kSampleSize);
  for (std::size_t i = 0; i < sample.size(); ++i) {
    sample[i] = static_cast<uint8_t>(i * 7);
  }
  const auto fragment = [&](uint32_t sn, uint32_t first, uint16_t count) {
    const std::size_t offset = static_cast<std::size_t>(first - 1) * kFragmentSize;
    reader.newFragment(writer_guid, rtps::SequenceNumber_t{0, sn}, first, count, kFragmentSize,
                       kSampleSize, sample.data() + offset,
                       static_cast<rtps::DataSize_t>(sample.size() - offset));
  };
  int32_t hb_count = 0;
  const auto heartbeat = [&](uint32_t first, uint32_t last) {
    rtps::SubmessageHeartbeat msg{};
    msg.header.submessageId = rtps::SubmessageKind::HEARTBEAT;
    msg.header.flags = rtps::FLAG_LITTLE_ENDIAN;
    msg.readerId = attributes.endpointGuid.entityId;
    msg.writerId = writer_guid.entityId;
    msg.firstSN = rtps::SequenceNumber_t{0, first};
    msg.lastSN = rtps::SequenceNumber_t{0, last};
    msg.count.value = ++hb_count;
    reader.onNewHeartbeat(msg, writer_guid.prefix);
  };
  const auto settle = [&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms + 20));
    Answer a;
    while (answer_arrived(sock, 20, a)) {
    }
  };
  Answer a;

  // 1. Fragment 1 held: the HEARTBEAT's answer asks for 2..22 by NACK_FRAG.
  fragment(1, 1, 1);
  heartbeat(1, 1);
  CHECK(answer_arrived(sock, 300, a), "the HEARTBEAT is answered");
  CHECK(a.nackfrag, "the answer carries a NACK_FRAG");
  CHECK(a.sn == 1, "... for sample 1");
  CHECK(a.base == 2 && a.nbits == kFragments - 1, "... from fragment 2, 21 bits");
  CHECK(a.word0 == 0xFFFFF800u, "... all 21 set"); // bits 0..20 of the word, MSB first
  settle();

  // 2. The rest arrives: delivered whole.
  fragment(1, 2, static_cast<uint16_t>(kFragments - 1));
  CHECK(delivered.count == 1 && delivered.size == kSampleSize, "the sample is delivered");

  // 3. Nothing under reassembly: no NACK_FRAG with the next answer.
  heartbeat(1, 1);
  CHECK(answer_arrived(sock, 300, a), "the next HEARTBEAT is answered");
  CHECK(!a.nackfrag, "... without a NACK_FRAG");
  settle();

  // 4. Fragments 1 and 5 of sample 2 held: 2..22 wanted but 5.
  fragment(2, 1, 1);
  fragment(2, 5, 1);
  heartbeat(1, 2);
  CHECK(answer_arrived(sock, 300, a), "answered with sample 2 partly held");
  CHECK(a.nackfrag && a.sn == 2 && a.base == 2 && a.nbits == kFragments - 1,
        "a NACK_FRAG for sample 2 from fragment 2");
  CHECK(a.word0 == (0xFFFFF800u & ~(uint32_t{1} << 28)), "... with fragment 5's bit clear");
  settle();

  // 5. A HEARTBEAT that does not announce sample 2 gets no NACK_FRAG for it.
  heartbeat(3, 2);
  CHECK(answer_arrived(sock, 300, a), "an empty HEARTBEAT past the held sample is answered");
  CHECK(!a.nackfrag, "... without a NACK_FRAG for a sample it does not announce");

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
