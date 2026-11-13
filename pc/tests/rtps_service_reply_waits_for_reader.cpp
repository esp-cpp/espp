// A service reply waits for the requester's reply reader to be matched.
//
// A client sends its request as soon as ITS side has matched the service's two
// endpoints. The server's reply writer may learn of the client's reply reader
// (its SEDP record) a little after the request has been served - the QEMU
// interop matrix's service call under 15 % loss against rmw_cyclonedds_cpp
// did that one run in six, the record arriving 40-100 ms after the request. A
// reply written before that match is never sent to the reader (a writer sends
// new changes to the readers it has), and the first HEARTBEAT the reader then
// sees announces it as history, which a VOLATILE reader skips: the call timed
// out although it had been served. rmw_fastrtps and rmw_cyclonedds servers
// wait for that match before writing a response; so does this facade now,
// up to Config::service_reply_match_timeout.
//
// The match alone is not enough against Cyclone DDS: a VOLATILE Cyclone
// reader skips everything the first HEARTBEAT it PROCESSES announces, and
// under loss (the same QEMU case, one call in 25) the reply was written
// between the match and that heartbeat. Cyclone numbers the pre-emptive
// ACKNACK it sends on matching 0 and the ones answering heartbeats from 1, so
// the reply also waits for an ACKNACK numbered 1 or more (the reader's
// acknowledgement, ReaderProxy::acknowledged), and the writer heartbeats a
// new reader at once so there is one to answer.
//
// The server is a facade participant hosting a service; the "client" is a
// peer built from hand-crafted datagrams sent from a UDP socket that is also
// the peer's only locator, so it receives everything the server sends to it.
// Checks:
//  1. control: the request (whose related sample identity names a reply
//     reader not yet announced) is served within 1 s;
//  2. no reply DATA leaves the writer while the reader is unknown (300 ms);
//  3. once the reply reader is announced over SEDP, the reply writer
//     heartbeats it within 500 ms (an empty history: first 1, last 0) and
//     still sends no reply DATA;
//  4. a pre-emptive ACKNACK numbered 0 (Cyclone's) is answered with a
//     heartbeat within 500 ms (the empty history again: what a reader whose
//     match heartbeat was lost needs to complete the handshake) and does not
//     release the reply: no DATA for 250 ms. Fails with the match alone;
//  5. an ACKNACK numbered 1 (the answer to the heartbeat) does: the reply
//     DATA arrives within 1 s. Fails without the hold: the writer had no
//     reader when the reply was written, so only its heartbeats ever reach
//     the socket;
//  6. with the reader acknowledged, a second request's reply is written at
//     once;
//  7. a request naming a reader that is never announced is still answered:
//     nothing for 300 ms, then (kHoldTimeout, 2 s) the reply goes to the
//     readers the writer has, as before the hold existed.
//
// Exits 0 on success, 1 on the first failed check.

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "rtps/common/Locator.hpp"
#include "rtps/messages/MessageTypes.hpp"
#include "rtps/rpc/sample_identity.hpp"
#include "rtps/rpc/service_naming.hpp"
#include "rtps/utils/udpUtils.hpp"
#include "rtps_common.hpp"
#include "rtps_participant.hpp"

namespace {

constexpr std::array<uint8_t, 2> kFastDdsVendor{0x01, 0x0f};
constexpr const char *kService = "/reply_waits_for_reader";
constexpr const char *kType = "example_interfaces::srv::dds_::AddTwoInts";
constexpr uint8_t kReply[8] = {0, 1, 0, 0, 7, 0, 0, 0};
// The server's hold (Config::service_reply_match_timeout): long enough for
// checks 2-4 to run their course before it (about 1.6 s at most), short
// enough for check 7 to fit the test. Check 7's wait is derived from it.
constexpr std::chrono::milliseconds kHoldTimeout{2000};

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

// --- datagram builders (little-endian, as Fast DDS writes them) -------------

void push_u16(std::vector<uint8_t> &p, uint16_t v) {
  p.push_back(static_cast<uint8_t>(v & 0xFF));
  p.push_back(static_cast<uint8_t>(v >> 8));
}

void push_u32(std::vector<uint8_t> &p, uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    p.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
  }
}

void push_entity_id(std::vector<uint8_t> &p, const rtps::EntityId_t &id) {
  p.insert(p.end(), id.entityKey.begin(), id.entityKey.end());
  p.push_back(static_cast<uint8_t>(id.entityKind));
}

void push_guid(std::vector<uint8_t> &p, const rtps::Guid_t &guid) {
  p.insert(p.end(), guid.prefix.id.begin(), guid.prefix.id.end());
  push_entity_id(p, guid.entityId);
}

void push_param(std::vector<uint8_t> &p, uint16_t pid, const std::vector<uint8_t> &value) {
  push_u16(p, pid);
  push_u16(p, static_cast<uint16_t>(value.size()));
  p.insert(p.end(), value.begin(), value.end());
}

void push_sentinel(std::vector<uint8_t> &p) {
  push_param(p, static_cast<uint16_t>(rtps::SMElement::ParameterId::PID_SENTINEL), {});
}

std::vector<uint8_t> guid_value(const rtps::Guid_t &guid) {
  std::vector<uint8_t> v;
  push_guid(v, guid);
  return v;
}

std::vector<uint8_t> string_value(const std::string &s) {
  std::vector<uint8_t> v;
  const auto len = static_cast<uint32_t>(s.size() + 1);
  push_u32(v, len);
  v.insert(v.end(), s.begin(), s.end());
  v.push_back(0);
  while (v.size() % 4 != 0) {
    v.push_back(0);
  }
  return v;
}

std::vector<uint8_t> locator_value(const rtps::Ip4AddressBytes &ip, uint32_t port) {
  std::vector<uint8_t> v;
  push_u32(v, static_cast<uint32_t>(rtps::LocatorKind_t::LOCATOR_KIND_UDPv4));
  push_u32(v, port);
  for (int i = 0; i < 12; ++i) {
    v.push_back(0);
  }
  v.insert(v.end(), ip.begin(), ip.end());
  return v;
}

std::vector<uint8_t> encapsulation() {
  std::vector<uint8_t> p(rtps::SMElement::SCHEME_PL_CDR_LE.begin(),
                         rtps::SMElement::SCHEME_PL_CDR_LE.end());
  push_u16(p, 0); // options
  return p;
}

// The peer announces itself with every locator pointing at `port` on `ip`.
std::vector<uint8_t> spdp_payload(const rtps::GuidPrefix_t &prefix, const rtps::Ip4AddressBytes &ip,
                                  uint32_t port) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> p = encapsulation();
  push_param(p, static_cast<uint16_t>(ParameterId::PID_PROTOCOL_VERSION),
             {rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor, 0, 0});
  push_param(p, static_cast<uint16_t>(ParameterId::PID_VENDORID),
             {kFastDdsVendor[0], kFastDdsVendor[1], 0, 0});
  push_param(p, static_cast<uint16_t>(ParameterId::PID_PARTICIPANT_GUID),
             guid_value(rtps::Guid_t{prefix, rtps::ENTITYID_BUILD_IN_PARTICIPANT}));
  push_param(p, static_cast<uint16_t>(ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR),
             locator_value(ip, port));
  push_param(p, static_cast<uint16_t>(ParameterId::PID_DEFAULT_UNICAST_LOCATOR),
             locator_value(ip, port));
  {
    std::vector<uint8_t> lease;
    push_u32(lease, 100); // Duration_t: int32 seconds, uint32 fraction
    push_u32(lease, 0);
    push_param(p, static_cast<uint16_t>(ParameterId::PID_PARTICIPANT_LEASE_DURATION), lease);
  }
  {
    std::vector<uint8_t> set;
    push_u32(set, rtps::SMElement::DISC_BIE_PARTICIPANT_ANNOUNCER |
                      rtps::SMElement::DISC_BIE_PARTICIPANT_DETECTOR |
                      rtps::SMElement::DISC_BIE_PUBLICATION_ANNOUNCER |
                      rtps::SMElement::DISC_BIE_PUBLICATION_DETECTOR |
                      rtps::SMElement::DISC_BIE_SUBSCRIPTION_ANNOUNCER |
                      rtps::SMElement::DISC_BIE_SUBSCRIPTION_DETECTOR);
    push_param(p, static_cast<uint16_t>(ParameterId::PID_BUILTIN_ENDPOINT_SET), set);
  }
  push_sentinel(p);
  return p;
}

// An SEDP announcement: GUID, topic, type, RELIABLE, the peer's locator.
std::vector<uint8_t> sedp_payload(const rtps::Guid_t &endpoint, const std::string &topic,
                                  const std::string &type, const rtps::Ip4AddressBytes &ip,
                                  uint32_t port) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> p = encapsulation();
  push_param(p, static_cast<uint16_t>(ParameterId::PID_ENDPOINT_GUID), guid_value(endpoint));
  push_param(p, static_cast<uint16_t>(ParameterId::PID_TOPIC_NAME), string_value(topic));
  push_param(p, static_cast<uint16_t>(ParameterId::PID_TYPE_NAME), string_value(type));
  {
    std::vector<uint8_t> rel;
    push_u32(rel, static_cast<uint32_t>(rtps::ReliabilityKind_t::RELIABLE));
    push_u32(rel, 0); // max_blocking_time
    push_u32(rel, 0);
    push_param(p, static_cast<uint16_t>(ParameterId::PID_RELIABILITY), rel);
  }
  push_param(p, static_cast<uint16_t>(ParameterId::PID_UNICAST_LOCATOR), locator_value(ip, port));
  push_sentinel(p);
  return p;
}

// The inline QoS of a request: the related sample identity naming the reply
// reader the client wants its answer on (as rmw_fastrtps sends it).
std::vector<uint8_t> related_inline_qos(const rtps::Guid_t &reply_reader, uint32_t seq) {
  std::vector<uint8_t> value;
  push_guid(value, reply_reader);
  push_u32(value, 0);   // SequenceNumber_t.high
  push_u32(value, seq); // SequenceNumber_t.low
  std::vector<uint8_t> q;
  push_param(q, rtps::rpc::PID_RELATED_SAMPLE_IDENTITY, value);
  push_sentinel(q);
  return q;
}

// An RTPS message from `prefix` with one DATA submessage.
std::vector<uint8_t> data_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                  const rtps::EntityId_t &reader, uint32_t sn,
                                  const std::vector<uint8_t> &inline_qos,
                                  const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> m{'R',
                         'T',
                         'P',
                         'S',
                         rtps::PROTOCOLVERSION.major,
                         rtps::PROTOCOLVERSION.minor,
                         kFastDdsVendor[0],
                         kFastDdsVendor[1]};
  m.insert(m.end(), prefix.id.begin(), prefix.id.end());
  uint8_t flags = rtps::FLAG_LITTLE_ENDIAN;
  if (!inline_qos.empty()) {
    flags |= rtps::FLAG_INLINE_QOS;
  }
  if (!payload.empty()) {
    flags |= rtps::FLAG_DATA_PAYLOAD;
  }
  m.push_back(static_cast<uint8_t>(rtps::SubmessageKind::DATA));
  m.push_back(flags);
  push_u16(m, static_cast<uint16_t>(2 + 2 + 4 + 4 + 8 + inline_qos.size() + payload.size()));
  push_u16(m, 0);  // extraFlags
  push_u16(m, 16); // octetsToInlineQos
  push_entity_id(m, reader);
  push_entity_id(m, writer);
  push_u32(m, 0);  // SequenceNumber_t.high
  push_u32(m, sn); // SequenceNumber_t.low
  m.insert(m.end(), inline_qos.begin(), inline_qos.end());
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}

// An RTPS message from `prefix` with one ACKNACK submessage: an empty set at
// `base` (nothing missing, nothing requested), numbered `count`. Cyclone's
// pre-emptive ACKNACK is base 1 / count 0; its answer to a heartbeat
// announcing an empty history is base 1 / count 1.
std::vector<uint8_t> acknack_message(const rtps::GuidPrefix_t &prefix,
                                     const rtps::EntityId_t &reader, const rtps::EntityId_t &writer,
                                     uint32_t base, uint32_t count) {
  std::vector<uint8_t> m{'R',
                         'T',
                         'P',
                         'S',
                         rtps::PROTOCOLVERSION.major,
                         rtps::PROTOCOLVERSION.minor,
                         kFastDdsVendor[0],
                         kFastDdsVendor[1]};
  m.insert(m.end(), prefix.id.begin(), prefix.id.end());
  m.push_back(static_cast<uint8_t>(rtps::SubmessageKind::ACKNACK));
  m.push_back(rtps::FLAG_LITTLE_ENDIAN); // not final: keep the heartbeats coming
  push_u16(m, 4 + 4 + 8 + 4 + 4);
  push_entity_id(m, reader);
  push_entity_id(m, writer);
  push_u32(m, 0);    // SequenceNumberSet.base.high
  push_u32(m, base); // SequenceNumberSet.base.low
  push_u32(m, 0);    // numBits
  push_u32(m, count);
  return m;
}

// --- the peer's socket ------------------------------------------------------

// The facade's participant id is whichever was free when it started; its
// ports follow from it, so the peer's datagrams go to the first few.
constexpr int kParticipantIds = 8;

void send_to_ports(int sock, const rtps::Ip4AddressBytes &ip, bool metatraffic,
                   const std::vector<uint8_t> &message) {
  for (int pid = 0; pid < kParticipantIds; ++pid) {
    sockaddr_in to{};
    to.sin_family = AF_INET;
    std::memcpy(&to.sin_addr, ip.data(), 4);
    to.sin_port =
        htons(metatraffic ? rtps::getBuiltInUnicastPort(pid) : rtps::getUserUnicastPort(pid));
    ::sendto(sock, message.data(), message.size(), 0, reinterpret_cast<sockaddr *>(&to),
             sizeof(to));
  }
}

// What the socket received from a user-defined writer: a reply DATA (with the
// related sample identity's sequence number from its inline QoS) or a
// HEARTBEAT (with its range and the writer it came from).
enum class Kind { None, Data, Heartbeat };
struct Seen {
  Kind kind{Kind::None};
  rtps::GuidPrefix_t prefix{}; ///< the server's
  rtps::EntityId_t writer{};
  uint32_t first{0}; ///< HEARTBEAT
  uint32_t last{0};
  uint32_t related_seq{0}; ///< DATA
  std::vector<uint8_t> payload;
};

// The wire fields are little-endian (the flag every submessage this engine
// sends on a little-endian host carries; the parser checks it and reads
// explicitly, so the test does not depend on the host's byte order).
uint32_t read_u32_le(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
uint16_t read_u16_le(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

bool user_writer(uint8_t kind) {
  return kind == static_cast<uint8_t>(rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY) ||
         kind == static_cast<uint8_t>(rtps::EntityKind_t::USER_DEFINED_WRITER_WITH_KEY);
}

// Scan one datagram for the first DATA or HEARTBEAT submessage of a
// user-defined writer.
Kind parse(const uint8_t *buf, std::size_t n, Seen &seen) {
  if (n < 20 || std::memcmp(buf, "RTPS", 4) != 0) {
    return Kind::None;
  }
  std::size_t pos = 20;
  while (pos + 4 <= n) {
    const uint8_t id = buf[pos];
    const uint8_t flags = buf[pos + 1];
    const bool little_endian = (flags & rtps::FLAG_LITTLE_ENDIAN) != 0;
    // Both lengths are endianness-flagged; without the flag the submessage
    // cannot be walked, so the datagram is given up on.
    if (!little_endian) {
      break;
    }
    const uint16_t o2n = read_u16_le(buf + pos + 2);
    const std::size_t body = pos + 4;
    const std::size_t end = o2n == 0 ? n : std::min(n, body + o2n);
    if (id == static_cast<uint8_t>(rtps::SubmessageKind::HEARTBEAT) && body + 28 <= end &&
        user_writer(buf[body + 7])) {
      seen = Seen{};
      seen.kind = Kind::Heartbeat;
      std::memcpy(seen.prefix.id.data(), buf + 8, seen.prefix.id.size());
      std::memcpy(seen.writer.entityKey.data(), buf + body + 4, 3);
      seen.writer.entityKind = static_cast<rtps::EntityKind_t>(buf[body + 7]);
      seen.first = read_u32_le(buf + body + 12);
      seen.last = read_u32_le(buf + body + 20);
      return seen.kind;
    }
    if (id == static_cast<uint8_t>(rtps::SubmessageKind::DATA) && body + 20 <= end &&
        user_writer(buf[body + 11])) {
      const uint16_t o2iq = read_u16_le(buf + body + 2);
      std::size_t cursor = body + 4 + o2iq;
      seen = Seen{};
      seen.kind = Kind::Data;
      std::memcpy(seen.prefix.id.data(), buf + 8, seen.prefix.id.size());
      std::memcpy(seen.writer.entityKey.data(), buf + body + 8, 3);
      seen.writer.entityKind = static_cast<rtps::EntityKind_t>(buf[body + 11]);
      if ((flags & rtps::FLAG_INLINE_QOS) != 0) {
        while (cursor + 4 <= end) {
          const uint16_t pid = read_u16_le(buf + cursor);
          const uint16_t len = read_u16_le(buf + cursor + 2);
          cursor += 4;
          if (pid == static_cast<uint16_t>(rtps::SMElement::ParameterId::PID_SENTINEL)) {
            break;
          }
          if ((pid == rtps::rpc::PID_RELATED_SAMPLE_IDENTITY ||
               pid == rtps::rpc::PID_CUSTOM_RELATED_SAMPLE_IDENTITY) &&
              len >= rtps::rpc::SAMPLE_IDENTITY_CDR_SIZE && cursor + 24 <= end) {
            seen.related_seq = read_u32_le(buf + cursor + 20);
          }
          cursor += len;
        }
      }
      if ((flags & rtps::FLAG_DATA_PAYLOAD) != 0 && cursor <= end) {
        seen.payload.assign(buf + cursor, buf + end);
      }
      return seen.kind;
    }
    if (o2n == 0) {
      break;
    }
    pos = body + o2n;
  }
  return Kind::None;
}

// Drain the socket for up to `wait_ms`: true when a submessage of kind `want`
// arrived (in `seen`, which is left untouched otherwise). Submessages of the
// other kind are counted in `others` and skipped.
bool arrived(int sock, int wait_ms, Kind want, Seen &seen, int *others = nullptr) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
  uint8_t buf[2048];
  while (std::chrono::steady_clock::now() < deadline) {
    const ssize_t n = ::recv(sock, buf, sizeof(buf), 0);
    if (n <= 0) {
      continue; // timeout tick
    }
    Seen parsed;
    const Kind kind = parse(buf, static_cast<std::size_t>(n), parsed);
    if (kind == want) {
      seen = std::move(parsed);
      return true;
    }
    if (kind != Kind::None && others != nullptr) {
      ++*others;
    }
  }
  return false;
}

bool reply_arrived(int sock, int wait_ms, Seen &seen) {
  return arrived(sock, wait_ms, Kind::Data, seen);
}

bool run_checks() {
  const std::string ip_str = rtps_test::guess_local_ipv4();
  unsigned a = 0, b = 0, c = 0, d = 0;
  CHECK(std::sscanf(ip_str.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4, "local IPv4");
  const rtps::Ip4AddressBytes ip{static_cast<uint8_t>(a), static_cast<uint8_t>(b),
                                 static_cast<uint8_t>(c), static_cast<uint8_t>(d)};

  // The peer's socket: bound to an ephemeral port, every locator it announces.
  const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  CHECK(sock >= 0, "socket");
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = 0;
  CHECK(::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0, "bind");
  socklen_t len = sizeof(addr);
  CHECK(::getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) == 0, "getsockname");
  const uint32_t port = ntohs(addr.sin_port);
  timeval tv{0, 50000};
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  espp::RtpsParticipant server({.interface_address = ip_str,
                                .log_level = espp::Logger::Verbosity::WARN,
                                .service_reply_match_timeout = kHoldTimeout});
  CHECK(server.start(), "server start");
  std::atomic<int> served{0};
  CHECK(server.add_service_server({kService, kType},
                                  [&](std::span<const uint8_t>) -> std::vector<uint8_t> {
                                    ++served;
                                    return std::vector<uint8_t>(kReply, kReply + sizeof(kReply));
                                  }),
        "add_service_server");

  // The peer appears, with its request writer.
  const rtps::GuidPrefix_t peer{
      {kFastDdsVendor[0], kFastDdsVendor[1], 0x5e, 0x55, 0x10, 0x4e, 0, 0, 0, 0, 0, 1}};
  const rtps::Guid_t request_writer{
      peer, {{0x00, 0x00, 0x01}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  const rtps::Guid_t reply_reader{
      peer, {{0x00, 0x00, 0x02}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  const rtps::Guid_t never_announced{
      peer, {{0x00, 0x00, 0x77}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  send_to_ports(sock, ip, true,
                data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                             rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, {},
                             spdp_payload(peer, ip, port)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  send_to_ports(
      sock, ip, true,
      data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                   rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, 1, {},
                   sedp_payload(request_writer, rtps::rpc::service_request_topic(kService),
                                rtps::rpc::service_request_type(kType), ip, port)));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  const uint8_t request[8] = {0, 1, 0, 0, 3, 0, 0, 0};
  const auto send_request = [&](uint32_t sn, const rtps::Guid_t &reader) {
    send_to_ports(sock, ip, false,
                  data_message(peer, request_writer.entityId, rtps::ENTITYID_UNKNOWN, sn,
                               related_inline_qos(reader, sn),
                               std::vector<uint8_t>(request, request + sizeof(request))));
  };

  // 1. Served, although the reply reader is not known yet.
  send_request(1, reply_reader);
  const auto served_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (served.load() < 1 && std::chrono::steady_clock::now() < served_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  CHECK(served.load() == 1, "the request was served");

  // 2. The reply is held: nothing from the reply writer in 300 ms.
  Seen seen;
  CHECK(!reply_arrived(sock, 300, seen), "no reply DATA while the reply reader is unknown");

  // 3. The reply reader is announced: the reply writer heartbeats it at once,
  //    announcing nothing (the reply is still held).
  send_to_ports(sock, ip, true,
                data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                             rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 1, {},
                             sedp_payload(reply_reader, rtps::rpc::service_reply_topic(kService),
                                          rtps::rpc::service_response_type(kType), ip, port)));
  int data_seen = 0;
  CHECK(arrived(sock, 500, Kind::Heartbeat, seen, &data_seen),
        "the reply writer heartbeats the reader once it is matched");
  CHECK(data_seen == 0, "no reply DATA with the match alone");
  CHECK(seen.first == 1 && seen.last == 0, "the heartbeat announces an empty history");
  const rtps::EntityId_t reply_writer = seen.writer;

  // 4. Cyclone's pre-emptive ACKNACK (count 0) is answered with a heartbeat
  //    (the writer's next periodic one is seconds away, so this is the answer)
  //    and does not release the reply.
  send_to_ports(sock, ip, false,
                acknack_message(peer, reply_reader.entityId, reply_writer, /*base=*/1,
                                /*count=*/0));
  data_seen = 0;
  CHECK(arrived(sock, 500, Kind::Heartbeat, seen, &data_seen),
        "a pre-emptive ACKNACK numbered 0 is answered with a heartbeat");
  CHECK(seen.first == 1 && seen.last == 0, "... announcing the empty history");
  CHECK(data_seen == 0 && !reply_arrived(sock, 250, seen),
        "no reply DATA on a pre-emptive ACKNACK numbered 0");

  // 5. The ACKNACK answering the heartbeat (count 1) does.
  send_to_ports(sock, ip, false,
                acknack_message(peer, reply_reader.entityId, reply_writer, /*base=*/1,
                                /*count=*/1));
  CHECK(reply_arrived(sock, 1000, seen), "the reply arrives once the reader has acknowledged");
  CHECK(seen.related_seq == 1, "it is the reply to request 1");
  CHECK(seen.payload.size() == sizeof(kReply) &&
            std::memcmp(seen.payload.data(), kReply, sizeof(kReply)) == 0,
        "it carries the handler's response");

  // 6. With the reader acknowledged, a reply is written at once.
  send_request(2, reply_reader);
  CHECK(reply_arrived(sock, 1000, seen),
        "a request with the reader acknowledged is answered at once");
  CHECK(seen.related_seq == 2, "it is the reply to request 2");

  // 7. A reader that never appears: held for the timeout, then written anyway.
  send_request(3, never_announced);
  CHECK(!reply_arrived(sock, 300, seen), "a reply for an unknown reader is held");
  CHECK(reply_arrived(sock, static_cast<int>(kHoldTimeout.count()) + 500, seen),
        "... and written once the hold times out");
  CHECK(seen.related_seq == 3, "it is the reply to request 3");

  server.stop();
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
