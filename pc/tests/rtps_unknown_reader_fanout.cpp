// Two local readers on one topic both get the writer, and both get the DATA.
//
// A writer may leave the reader unnamed (readerId ENTITYID_UNKNOWN, RTPS 2.5
// 8.3.7.2): every multicast DATA, and a DATA sent to several readers at once,
// is addressed that way, as are Cyclone DDS's HEARTBEATs and GAPs. The
// receiver resolved such a submessage through the writer to ONE local reader
// (Participant::getReaderByWriterId returned the first with the proxy), and
// SEDP itself matched a remote writer to ONE local reader on the topic
// (getMatchingReader, the first) - so of two readers on a topic in one
// participant, which the facade allows, only the first ever saw anything, and
// the second's reliable state never advanced either. Now SEDP matches every
// local endpoint on the topic (Participant::getMatchingReaders /
// getMatchingWriters) and the receiver fans an unnamed submessage out to every
// reader matched to the writer (getReadersByWriterId / resolveReaders).
//
// Hand-crafted datagrams through Participant::newMessage; no network:
//  1. Two best-effort readers on one topic; one remote writer announced over
//     SEDP: both readers hold its proxy.
//  2. A DATA with readerId ENTITYID_UNKNOWN reaches both callbacks; a DATA
//     naming one reader reaches that one only.
//  3. Two local writers on one topic and one remote reader announced: both
//     writers hold its proxy.
//  4. Two reliable readers, a GAP with readerId ENTITYID_UNKNOWN jumping both
//     over SN 1..2, then an unnamed DATA with SN 3: delivered to both (a
//     reliable reader only takes the SN it expects, so the GAP reached both).
//     A HEARTBEAT with readerId ENTITYID_UNKNOWN is taken without harm.
//  5. Late creation: a peer announces an endpoint once (SEDP is reliable, no
//     periodic resend), so a local endpoint created AFTER the announcement
//     can only match it from the agent's registry of known remote endpoints.
//     That registry used to keep just the endpoints that matched nothing: a
//     remote writer that had matched reader A was invisible to a reader B
//     created later. Now a reader B on the topic gets the writer at creation,
//     a late local writer gets the remote reader, and a remote writer
//     disposed before the late creation is not matched.
//
// Exits 0 on success, 1 on the first failed check.

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

std::vector<uint8_t> header(const rtps::GuidPrefix_t &prefix) {
  std::vector<uint8_t> m{
      'R',        'T',       'P', 'S', rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor,
      kVendor[0], kVendor[1]};
  m.insert(m.end(), prefix.id.begin(), prefix.id.end());
  return m;
}

void push_submessage(std::vector<uint8_t> &m, uint8_t id, uint8_t flags,
                     const std::vector<uint8_t> &body) {
  m.push_back(id);
  m.push_back(flags);
  push_u16(m, static_cast<uint16_t>(body.size()));
  m.insert(m.end(), body.begin(), body.end());
}

std::vector<uint8_t> data_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                  const rtps::EntityId_t &reader, uint32_t sn,
                                  const std::vector<uint8_t> &payload) {
  auto m = header(prefix);
  std::vector<uint8_t> b;
  push_u16(b, 0);
  push_u16(b, 16);
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, sn);
  b.insert(b.end(), payload.begin(), payload.end());
  push_submessage(m, static_cast<uint8_t>(rtps::SubmessageKind::DATA),
                  rtps::FLAG_LITTLE_ENDIAN | rtps::FLAG_DATA_PAYLOAD, b);
  return m;
}

// GAP: readerId, writerId, gapStart, gapList{base, numBits=0}: the numbers
// gapStart..base-1 are irrelevant to the reader.
std::vector<uint8_t> gap_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                 const rtps::EntityId_t &reader, uint32_t gap_start,
                                 uint32_t base) {
  auto m = header(prefix);
  std::vector<uint8_t> b;
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, gap_start);
  push_u32(b, 0);
  push_u32(b, base);
  push_u32(b, 0);
  push_submessage(m, static_cast<uint8_t>(rtps::SubmessageKind::GAP), rtps::FLAG_LITTLE_ENDIAN, b);
  return m;
}

std::vector<uint8_t> heartbeat_message(const rtps::GuidPrefix_t &prefix,
                                       const rtps::EntityId_t &writer,
                                       const rtps::EntityId_t &reader, uint32_t first,
                                       uint32_t last, uint32_t count) {
  auto m = header(prefix);
  std::vector<uint8_t> b;
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, first);
  push_u32(b, 0);
  push_u32(b, last);
  push_u32(b, count);
  push_submessage(m, static_cast<uint8_t>(rtps::SubmessageKind::HEARTBEAT),
                  rtps::FLAG_LITTLE_ENDIAN | rtps::FLAG_FINAL, b);
  return m;
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

constexpr const char *kType = "FanoutType";

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");

  const rtps::GuidPrefix_t peer{{kVendor[0], kVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, 9}};
  feed(part, data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                          rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 1, "peer discovered");
  uint32_t pub_sn = 0;
  uint32_t sub_sn = 0;
  const std::vector<uint8_t> payload{0, 1, 0, 0, 1, 2, 3, 4};

  // 1. Two best-effort readers on one topic, one remote writer.
  rtps::Reader *be1 = domain.createReader(*part, "be_topic", kType, false, {0, 0, 0, 0});
  rtps::Reader *be2 = domain.createReader(*part, "be_topic", kType, false, {0, 0, 0, 0});
  CHECK(be1 != nullptr && be2 != nullptr, "two best-effort readers");
  Delivered d1;
  Delivered d2;
  CHECK(be1->registerCallback(&on_sample, &d1) != 0 && be2->registerCallback(&on_sample, &d2) != 0,
        "registerCallback");
  const rtps::Guid_t be_writer{
      peer, {{0x00, 0x00, 0x01}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn,
                          sedp_payload(be_writer, "be_topic", kType, false)));
  CHECK(be1->getProxiesCount() == 1 && be2->getProxiesCount() == 1,
        "the remote writer is matched to both readers on the topic");
  CHECK(be1->isProxy(be_writer) && be2->isProxy(be_writer), "and it is the announced writer");

  // 2. An unnamed DATA reaches both; a named one reaches that reader only.
  feed(part, data_message(peer, be_writer.entityId, rtps::ENTITYID_UNKNOWN, 1, payload));
  CHECK(d1.count == 1 && d2.count == 1, "DATA to ENTITYID_UNKNOWN reaches both readers");
  feed(part,
       data_message(peer, be_writer.entityId, be2->m_attributes.endpointGuid.entityId, 2, payload));
  CHECK(d1.count == 1 && d2.count == 2, "DATA naming the second reader reaches it alone");

  // 3. Two local writers, one remote reader.
  rtps::Writer *w1 = domain.createWriter(*part, "w_topic", kType, true, false);
  rtps::Writer *w2 = domain.createWriter(*part, "w_topic", kType, true, false);
  CHECK(w1 != nullptr && w2 != nullptr, "two writers");
  const rtps::Guid_t remote_reader{
      peer, {{0x00, 0x00, 0x02}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, ++sub_sn,
                          sedp_payload(remote_reader, "w_topic", kType, true)));
  CHECK(w1->getProxiesCount() == 1 && w2->getProxiesCount() == 1,
        "the remote reader is matched to both writers on the topic");

  // 4. Two reliable readers: an unnamed GAP advances both, an unnamed DATA at
  //    the new expected SN is delivered to both; an unnamed HEARTBEAT is fine.
  rtps::Reader *r1 = domain.createReader(*part, "rel_topic", kType, true, {0, 0, 0, 0});
  rtps::Reader *r2 = domain.createReader(*part, "rel_topic", kType, true, {0, 0, 0, 0});
  CHECK(r1 != nullptr && r2 != nullptr, "two reliable readers");
  Delivered e1;
  Delivered e2;
  CHECK(r1->registerCallback(&on_sample, &e1) != 0 && r2->registerCallback(&on_sample, &e2) != 0,
        "registerCallback (reliable)");
  const rtps::Guid_t rel_writer{
      peer, {{0x00, 0x00, 0x03}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn,
                          sedp_payload(rel_writer, "rel_topic", kType, true)));
  CHECK(r1->getProxiesCount() == 1 && r2->getProxiesCount() == 1,
        "the reliable remote writer is matched to both reliable readers");
  feed(part, heartbeat_message(peer, rel_writer.entityId, rtps::ENTITYID_UNKNOWN, 1, 2, 1));
  feed(part, data_message(peer, rel_writer.entityId, rtps::ENTITYID_UNKNOWN, 3, payload));
  CHECK(e1.count == 0 && e2.count == 0, "SN 3 is not expected yet by either reliable reader");
  feed(part, gap_message(peer, rel_writer.entityId, rtps::ENTITYID_UNKNOWN, 1, 3));
  feed(part, data_message(peer, rel_writer.entityId, rtps::ENTITYID_UNKNOWN, 3, payload));
  CHECK(e1.count == 1 && e2.count == 1,
        "after the unnamed GAP both reliable readers take the unnamed DATA at SN 3");

  // 5. Late creation. be_writer already matched be1 and be2; a reader created
  //    now must get it from the registry.
  rtps::Reader *be3 = domain.createReader(*part, "be_topic", kType, false, {0, 0, 0, 0});
  CHECK(be3 != nullptr, "a third best-effort reader, created after the announcement");
  CHECK(be3->getProxiesCount() == 1 && be3->isProxy(be_writer),
        "a reader created after the announcement is matched to the remembered writer");
  Delivered d3;
  CHECK(be3->registerCallback(&on_sample, &d3) != 0, "registerCallback (late reader)");
  feed(part, data_message(peer, be_writer.entityId, rtps::ENTITYID_UNKNOWN, 3, payload));
  CHECK(d1.count == 2 && d2.count == 3 && d3.count == 1,
        "an unnamed DATA reaches the late reader too");
  // remote_reader already matched w1 and w2; a writer created now gets it.
  rtps::Writer *w3 = domain.createWriter(*part, "w_topic", kType, true, false);
  CHECK(w3 != nullptr, "a third writer, created after the announcement");
  CHECK(w3->getProxiesCount() == 1 && w3->isProxy(remote_reader),
        "a writer created after the announcement is matched to the remembered reader");
  // A remote writer disposed before the late creation is forgotten: a reader
  // created afterwards does not get it.
  const rtps::Guid_t gone_writer{
      peer, {{0x00, 0x00, 0x04}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn,
                          sedp_payload(gone_writer, "gone_topic", kType, false)));
  {
    using rtps::SMElement::ParameterId;
    auto m = header(peer);
    std::vector<uint8_t> b;
    push_u16(b, 0);
    push_u16(b, 16);
    push_entity_id(b, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER);
    push_entity_id(b, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER);
    push_u32(b, 0);
    push_u32(b, ++pub_sn);
    push_param(b, ParameterId::PID_KEY_HASH, guid_value(gone_writer)); // Fast DDS dispose form
    push_param(b, ParameterId::PID_STATUS_INFO, {0, 0, 0, 1});
    push_sentinel(b);
    push_submessage(m, static_cast<uint8_t>(rtps::SubmessageKind::DATA),
                    rtps::FLAG_LITTLE_ENDIAN | rtps::FLAG_INLINE_QOS, b);
    feed(part, m);
  }
  rtps::Reader *gone = domain.createReader(*part, "gone_topic", kType, false, {0, 0, 0, 0});
  CHECK(gone != nullptr, "a reader on the disposed writer's topic");
  CHECK(gone->getProxiesCount() == 0,
        "a remote writer disposed before the reader was created is not matched");
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
