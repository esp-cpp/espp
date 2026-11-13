// Malformed submessages are rejected, not parsed past their bytes.
//
// The ACKNACK and GAP deserializers derived their bitmap length from the
// submessage's octetsToNextHeader without checking it against the fixed
// fields or the datagram: an ACKNACK shorter than its fixed body underflowed
// the length (size_t), the set parser copied 32 bytes past the submessage and
// then moved the cursor by the wrapped amount, and the count was read from a
// wild pointer. A numBits above the set's 256 made the writer treat every
// number past the set as requested. A peer bug, or any host on the LAN,
// could crash the device. Now the fixed fields, the declared length against
// the datagram, and the bitmap bytes against numBits are all checked
// (deserializeMessage for ACKNACK / GAP / HEARTBEAT, deserializeSNS).
//
// The checks hand crafted datagrams to a participant through its receive
// entry point (Participant::newMessage); no network, no second process. A
// peer is discovered over SPDP, a remote reader is matched to a local
// reliable writer (so ACKNACKs resolve) and a remote writer to a local
// best-effort reader (so DATAs are delivered). Then a table of malformed
// submessages goes in - each must be ignored without a crash - and a valid
// ACKNACK and a valid DATA afterwards must still take effect:
//  1. ACKNACK declared shorter than its fixed body (the size_t underflow).
//  2. ACKNACK declared longer than the datagram.
//  3. ACKNACK with numBits 0xFFFFFFFF and a 4-byte bitmap.
//  4. GAP declared longer than the datagram.
//  5. HEARTBEAT declared shorter than its body; DATA and DATA_FRAG likewise.
//  6. A datagram truncated in the middle of a DATA.
//  7. An unknown submessage id followed by a valid DATA: the DATA is delivered.
//  8. A valid ACKNACK acknowledges the proxy; a valid DATA reaches the reader.
//
// Exits 0 on success, 1 on the first failed check (a crash is a failure too).

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/messages/MessageTypes.hpp"

namespace {

constexpr rtps::Ip4AddressBytes kIp{127, 0, 0, 1};
constexpr std::array<uint8_t, 2> kVendor{0x01, 0x0f};

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

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

void push_param(std::vector<uint8_t> &p, rtps::SMElement::ParameterId pid,
                const std::vector<uint8_t> &value) {
  push_u16(p, static_cast<uint16_t>(pid));
  push_u16(p, static_cast<uint16_t>(value.size()));
  p.insert(p.end(), value.begin(), value.end());
}

void push_sentinel(std::vector<uint8_t> &p) {
  push_param(p, rtps::SMElement::ParameterId::PID_SENTINEL, {});
}

std::vector<uint8_t> guid_value(const rtps::Guid_t &guid) {
  std::vector<uint8_t> v;
  push_guid(v, guid);
  return v;
}

std::vector<uint8_t> string_value(const char *s) {
  std::vector<uint8_t> v;
  const auto len = static_cast<uint32_t>(std::strlen(s) + 1);
  push_u32(v, len);
  v.insert(v.end(), s, s + len);
  while (v.size() % 4 != 0) {
    v.push_back(0);
  }
  return v;
}

std::vector<uint8_t> locator_value(uint32_t port) {
  std::vector<uint8_t> v;
  push_u32(v, static_cast<uint32_t>(rtps::LocatorKind_t::LOCATOR_KIND_UDPv4));
  push_u32(v, port);
  for (int i = 0; i < 12; ++i) {
    v.push_back(0);
  }
  v.insert(v.end(), kIp.begin(), kIp.end());
  return v;
}

std::vector<uint8_t> encapsulation() {
  std::vector<uint8_t> p(rtps::SMElement::SCHEME_PL_CDR_LE.begin(),
                         rtps::SMElement::SCHEME_PL_CDR_LE.end());
  push_u16(p, 0);
  return p;
}

std::vector<uint8_t> spdp_payload(const rtps::GuidPrefix_t &prefix) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> p = encapsulation();
  push_param(p, ParameterId::PID_PROTOCOL_VERSION,
             {rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor, 0, 0});
  push_param(p, ParameterId::PID_VENDORID, {kVendor[0], kVendor[1], 0, 0});
  push_param(p, ParameterId::PID_PARTICIPANT_GUID,
             guid_value(rtps::Guid_t{prefix, rtps::ENTITYID_BUILD_IN_PARTICIPANT}));
  push_param(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, locator_value(7410));
  push_param(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, locator_value(7411));
  {
    std::vector<uint8_t> lease;
    push_u32(lease, 100);
    push_u32(lease, 0);
    push_param(p, ParameterId::PID_PARTICIPANT_LEASE_DURATION, lease);
  }
  {
    std::vector<uint8_t> set;
    push_u32(set, rtps::SMElement::DISC_BIE_PARTICIPANT_ANNOUNCER |
                      rtps::SMElement::DISC_BIE_PARTICIPANT_DETECTOR |
                      rtps::SMElement::DISC_BIE_PUBLICATION_ANNOUNCER |
                      rtps::SMElement::DISC_BIE_PUBLICATION_DETECTOR |
                      rtps::SMElement::DISC_BIE_SUBSCRIPTION_ANNOUNCER |
                      rtps::SMElement::DISC_BIE_SUBSCRIPTION_DETECTOR);
    push_param(p, ParameterId::PID_BUILTIN_ENDPOINT_SET, set);
  }
  push_sentinel(p);
  return p;
}

std::vector<uint8_t> sedp_payload(const rtps::Guid_t &endpoint, const char *topic, const char *type,
                                  bool reliable) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> p = encapsulation();
  push_param(p, ParameterId::PID_ENDPOINT_GUID, guid_value(endpoint));
  push_param(p, ParameterId::PID_TOPIC_NAME, string_value(topic));
  push_param(p, ParameterId::PID_TYPE_NAME, string_value(type));
  {
    std::vector<uint8_t> rel;
    push_u32(rel, static_cast<uint32_t>(reliable ? rtps::ReliabilityKind_t::RELIABLE
                                                 : rtps::ReliabilityKind_t::BEST_EFFORT));
    push_u32(rel, 0);
    push_u32(rel, 0);
    push_param(p, ParameterId::PID_RELIABILITY, rel);
  }
  push_param(p, ParameterId::PID_UNICAST_LOCATOR, locator_value(7411));
  push_sentinel(p);
  return p;
}

// The RTPS header from `prefix`.
std::vector<uint8_t> header(const rtps::GuidPrefix_t &prefix) {
  std::vector<uint8_t> m{
      'R',        'T',       'P', 'S', rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor,
      kVendor[0], kVendor[1]};
  m.insert(m.end(), prefix.id.begin(), prefix.id.end());
  return m;
}

// One submessage: id, flags, the DECLARED length, then the body as given (so
// the declared length and the real one can disagree on purpose).
void push_submessage(std::vector<uint8_t> &m, uint8_t id, uint8_t flags, uint16_t declared,
                     const std::vector<uint8_t> &body) {
  m.push_back(id);
  m.push_back(flags);
  push_u16(m, declared);
  m.insert(m.end(), body.begin(), body.end());
}

// A DATA body: extraFlags, octetsToInlineQos, readerId, writerId, SN, payload.
std::vector<uint8_t> data_body(const rtps::EntityId_t &writer, const rtps::EntityId_t &reader,
                               uint32_t sn, const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> b;
  push_u16(b, 0);
  push_u16(b, 16);
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, sn);
  b.insert(b.end(), payload.begin(), payload.end());
  return b;
}

std::vector<uint8_t> data_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                  const rtps::EntityId_t &reader, uint32_t sn,
                                  const std::vector<uint8_t> &payload) {
  auto m = header(prefix);
  const auto body = data_body(writer, reader, sn, payload);
  push_submessage(m, static_cast<uint8_t>(rtps::SubmessageKind::DATA),
                  rtps::FLAG_LITTLE_ENDIAN | rtps::FLAG_DATA_PAYLOAD,
                  static_cast<uint16_t>(body.size()), body);
  return m;
}

// An ACKNACK body: readerId, writerId, SequenceNumberSet{base, numBits,
// bitmap words}, count.
std::vector<uint8_t> acknack_body(const rtps::EntityId_t &reader, const rtps::EntityId_t &writer,
                                  uint32_t base, uint32_t num_bits,
                                  const std::vector<uint32_t> &bitmap, uint32_t count) {
  std::vector<uint8_t> b;
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, base);
  push_u32(b, num_bits);
  for (auto w : bitmap) {
    push_u32(b, w);
  }
  push_u32(b, count);
  return b;
}

// A GAP body: readerId, writerId, gapStart, SequenceNumberSet.
std::vector<uint8_t> gap_body(const rtps::EntityId_t &reader, const rtps::EntityId_t &writer,
                              uint32_t gap_start, uint32_t base, uint32_t num_bits,
                              const std::vector<uint32_t> &bitmap) {
  std::vector<uint8_t> b;
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, gap_start);
  push_u32(b, 0);
  push_u32(b, base);
  push_u32(b, num_bits);
  for (auto w : bitmap) {
    push_u32(b, w);
  }
  return b;
}

// A HEARTBEAT body: readerId, writerId, firstSN, lastSN, count.
std::vector<uint8_t> heartbeat_body(const rtps::EntityId_t &reader, const rtps::EntityId_t &writer,
                                    uint32_t first, uint32_t last, uint32_t count) {
  std::vector<uint8_t> b;
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, first);
  push_u32(b, 0);
  push_u32(b, last);
  push_u32(b, count);
  return b;
}

void feed(rtps::Participant *part, const std::vector<uint8_t> &message) {
  part->newMessage(message.data(), static_cast<rtps::DataSize_t>(message.size()));
}

struct Delivered {
  int count{0};
};

void on_sample(void *arg, const rtps::ReaderCacheChange &) {
  static_cast<Delivered *>(arg)->count++;
}

struct Acked {
  bool acknowledged{false};
};

void collect_ack(const rtps::Writer *, const rtps::ReaderProxy &proxy, void *arg) {
  if (proxy.acknowledged) {
    static_cast<Acked *>(arg)->acknowledged = true;
  }
}

constexpr const char *kTopic = "malformed_topic";
constexpr const char *kType = "MalformedType";

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");

  rtps::Reader *reader =
      domain.createReader(*part, kTopic, kType, /*reliable=*/false, {0, 0, 0, 0});
  rtps::Writer *writer = domain.createWriter(*part, kTopic, kType, /*reliable=*/true,
                                             /*enforceUnicast=*/false);
  CHECK(reader != nullptr && writer != nullptr, "local endpoints");
  Delivered delivered;
  CHECK(reader->registerCallback(&on_sample, &delivered) != 0, "registerCallback");
  const rtps::EntityId_t local_reader = reader->m_attributes.endpointGuid.entityId;
  const rtps::EntityId_t local_writer = writer->m_attributes.endpointGuid.entityId;

  const rtps::GuidPrefix_t peer{{kVendor[0], kVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, 7}};
  feed(part, data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                          rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 1, "peer discovered");

  const rtps::Guid_t remote_writer{
      peer, {{0x00, 0x00, 0x01}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  const rtps::Guid_t remote_reader{
      peer, {{0x00, 0x00, 0x02}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, 1,
                          sedp_payload(remote_writer, kTopic, kType, false)));
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 1,
                          sedp_payload(remote_reader, kTopic, kType, true)));
  CHECK(reader->getProxiesCount() == 1, "remote writer matched to the local reader");
  CHECK(writer->getProxiesCount() == 1, "remote reader matched to the local writer");

  // Something for the writer to have sent, so an ACKNACK walks real history.
  const uint8_t sample[8] = {0, 1, 0, 0, 1, 2, 3, 4};
  CHECK(writer->newChange(rtps::ChangeKind_t::ALIVE, sample, sizeof(sample)) != nullptr,
        "a change in the writer's history");

  const auto acknack = static_cast<uint8_t>(rtps::SubmessageKind::ACKNACK);
  const auto gap = static_cast<uint8_t>(rtps::SubmessageKind::GAP);
  const auto heartbeat = static_cast<uint8_t>(rtps::SubmessageKind::HEARTBEAT);
  const auto data = static_cast<uint8_t>(rtps::SubmessageKind::DATA);
  const auto data_frag = static_cast<uint8_t>(rtps::SubmessageKind::DATA_FRAG);
  const uint8_t le = rtps::FLAG_LITTLE_ENDIAN;
  const uint8_t le_final = rtps::FLAG_LITTLE_ENDIAN | rtps::FLAG_FINAL;

  // 1. ACKNACK declared shorter than its fixed body (24): the length underflow.
  {
    auto m = header(peer);
    const auto body = acknack_body(remote_reader.entityId, local_writer, 2, 0, {}, 1);
    push_submessage(m, acknack, le_final, 20, body);
    feed(part, m);
  }
  // 2. ACKNACK declared longer than the datagram.
  {
    auto m = header(peer);
    const auto body = acknack_body(remote_reader.entityId, local_writer, 2, 0, {}, 2);
    push_submessage(m, acknack, le_final, 200, body);
    feed(part, m);
  }
  // 3. numBits 0xFFFFFFFF with a single bitmap word.
  {
    auto m = header(peer);
    const auto body =
        acknack_body(remote_reader.entityId, local_writer, 1, 0xFFFFFFFFu, {0x80000000u}, 3);
    push_submessage(m, acknack, le_final, static_cast<uint16_t>(body.size()), body);
    feed(part, m);
  }
  // 4. GAP declared longer than the datagram; and a GAP with numBits past the set.
  {
    auto m = header(peer);
    const auto body = gap_body(local_reader, remote_writer.entityId, 1, 2, 0, {});
    push_submessage(m, gap, le, 300, body);
    feed(part, m);
    auto m2 = header(peer);
    const auto body2 = gap_body(local_reader, remote_writer.entityId, 1, 2, 4000, {0xFFFFFFFFu});
    push_submessage(m2, gap, le, static_cast<uint16_t>(body2.size()), body2);
    feed(part, m2);
  }
  // 5. HEARTBEAT, DATA and DATA_FRAG declared shorter than their fixed bodies.
  {
    auto m = header(peer);
    const auto body = heartbeat_body(local_reader, remote_writer.entityId, 1, 1, 1);
    push_submessage(m, heartbeat, le_final, 8, body);
    feed(part, m);
    auto m2 = header(peer);
    const auto body2 = data_body(remote_writer.entityId, local_reader, 1, {0, 1, 0, 0, 1, 2, 3, 4});
    push_submessage(m2, data, le | rtps::FLAG_DATA_PAYLOAD, 6, body2);
    feed(part, m2);
    auto m3 = header(peer);
    std::vector<uint8_t> frag_body;
    push_u16(frag_body, 0);
    push_u16(frag_body, 28);
    push_entity_id(frag_body, local_reader);
    push_entity_id(frag_body, remote_writer.entityId);
    push_u32(frag_body, 0);
    push_u32(frag_body, 1);
    push_submessage(m3, data_frag, le, 12, frag_body); // cut before fragmentStartingNum
    feed(part, m3);
  }
  // 6. A datagram truncated in the middle of a DATA.
  {
    auto m = data_message(peer, remote_writer.entityId, local_reader, 1,
                          {0, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8});
    m.resize(m.size() - 10);
    feed(part, m);
  }
  CHECK(delivered.count == 0, "nothing malformed was delivered");
  {
    // The old parser took the short and the oversized ACKNACKs as valid (and
    // read past them): the proxy came out acknowledged. A rejected ACKNACK
    // must leave it untouched.
    Acked acked;
    writer->dumpAllProxies(&collect_ack, &acked);
    CHECK(!acked.acknowledged, "no malformed ACKNACK acknowledged the proxy");
  }

  // 7. An unknown submessage id, then a valid DATA in the same datagram.
  {
    auto m = header(peer);
    push_submessage(m, 0x7f, le, 4, {0xde, 0xad, 0xbe, 0xef});
    const auto body = data_body(remote_writer.entityId, local_reader, 1, {0, 1, 0, 0, 1, 2, 3, 4});
    push_submessage(m, data, le | rtps::FLAG_DATA_PAYLOAD, static_cast<uint16_t>(body.size()),
                    body);
    feed(part, m);
  }
  CHECK(delivered.count == 1, "the DATA after an unknown submessage is delivered");

  // 8. A valid ACKNACK still acknowledges, a valid DATA is still delivered.
  {
    auto m = header(peer);
    const auto body = acknack_body(remote_reader.entityId, local_writer, 2, 0, {}, 9);
    push_submessage(m, acknack, le_final, static_cast<uint16_t>(body.size()), body);
    feed(part, m);
    Acked acked;
    writer->dumpAllProxies(&collect_ack, &acked);
    CHECK(acked.acknowledged, "a valid ACKNACK after the malformed ones acknowledges the proxy");
  }
  feed(part, data_message(peer, remote_writer.entityId, local_reader, 2, {0, 1, 0, 0, 5, 6, 7, 8}));
  CHECK(delivered.count == 2, "a valid DATA after the malformed ones is delivered");
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
