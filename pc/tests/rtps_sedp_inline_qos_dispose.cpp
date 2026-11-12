// A peer's dispose of an endpoint or participant must unmatch it at once.
//
// When a DDS peer deletes a publisher/subscriber or leaves the domain it writes
// a DATA on the SEDP/SPDP writer whose inline QoS carries PID_STATUS_INFO (the
// change kind: disposed / unregistered, RTPS 2.5 9.6.3.9) and PID_KEY_HASH (the
// instance's GUID, 9.6.3.8), usually with no payload. Fast DDS, Cyclone DDS
// and this engine all announce deletions that way. The receiver used to walk
// the inline QoS only for the related-sample-identity and hand the readers the
// payload alone, so the discovery agents never saw a dispose: a deleted remote
// endpoint stayed matched (and a gone participant known) until the
// participant's lease ran out, 10-20 s with ROS 2 defaults, and a peer that
// restarted quickly was matched twice.
//
// The checks hand crafted datagrams to a participant through its receive
// entry point (Participant::newMessage); no network, no second process:
//  1. A remote writer announced over SEDP matches the local reader (control),
//     also when the announcement itself carries an inline QoS without a status
//     (Fast DDS attaches the key hash to announcements too): still ALIVE.
//  2. The Fast DDS form of its dispose - inline QoS with KEY_HASH + STATUS_INFO,
//     no payload - unmatches it. Fails without the fix.
//  3. The Cyclone DDS form - STATUS_INFO inline, the endpoint GUID in a PL_CDR
//     payload, no key hash - unmatches it too.
//  4. This engine's own form (SEDPAgent::announceEndpointDeletion: KEY_HASH +
//     STATUS_INFO inline, a bare sentinel as payload) unmatches it: espp
//     peers see each other's deletions.
//  5. A dispose for an endpoint nobody matched clears it from the known-endpoint
//     registry, and a dispose for an unknown GUID changes nothing.
//  6. A remote reader announced on the subscriptions writer matches the local
//     writer and its dispose unmatches it (the other SEDP leg).
//  7. Only the owner may dispose: a dispose from A naming B's endpoint or B's
//     participant, or one on the wrong SEDP leg, changes nothing.
//  8. The participant's own dispose over SPDP removes it, with the proxies
//     of its endpoints; an announcement delayed from before the dispose does
//     not bring it back, a later one does.
//  9. A payload-form participant dispose in big-endian PL_CDR is honoured;
//     one whose GUID names an endpoint rather than the participant is not.
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
constexpr std::array<uint8_t, 2> kFastDdsVendor{0x01, 0x0f};

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

// One parameter: pid, length, value (a multiple of 4 bytes long).
void push_param(std::vector<uint8_t> &p, rtps::SMElement::ParameterId pid,
                const std::vector<uint8_t> &value) {
  push_u16(p, static_cast<uint16_t>(pid));
  push_u16(p, static_cast<uint16_t>(value.size()));
  p.insert(p.end(), value.begin(), value.end());
}

void push_sentinel(std::vector<uint8_t> &p) {
  push_param(p, rtps::SMElement::ParameterId::PID_SENTINEL, {});
}

std::vector<uint8_t> sentinel() {
  std::vector<uint8_t> p;
  push_sentinel(p);
  return p;
}

std::vector<uint8_t> guid_value(const rtps::Guid_t &guid) {
  std::vector<uint8_t> v;
  push_guid(v, guid);
  return v;
}

// PID_STATUS_INFO: four octets, the flags in the last one (disposed = 0x01).
std::vector<uint8_t> status_value(uint8_t flags) { return {0, 0, 0, flags}; }

// A CDR string as SEDP carries it: uint32 length (with the NUL), the bytes, pad.
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
  push_u16(p, 0); // options
  return p;
}

// The SPDP payload a peer announces itself with (see rtps_spdp_unknown_reader).
std::vector<uint8_t> spdp_payload(const rtps::GuidPrefix_t &prefix) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> p = encapsulation();
  push_param(p, ParameterId::PID_PROTOCOL_VERSION,
             {rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor, 0, 0});
  push_param(p, ParameterId::PID_VENDORID, {kFastDdsVendor[0], kFastDdsVendor[1], 0, 0});
  push_param(p, ParameterId::PID_PARTICIPANT_GUID,
             guid_value(rtps::Guid_t{prefix, rtps::ENTITYID_BUILD_IN_PARTICIPANT}));
  push_param(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, locator_value(7410));
  push_param(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, locator_value(7411));
  {
    std::vector<uint8_t> lease;
    push_u32(lease, 100); // Duration_t: int32 seconds, uint32 fraction
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

// The SEDP payload announcing an endpoint: GUID, topic, type, reliability and
// a unicast locator, as Fast DDS writes it.
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
    push_u32(rel, 0); // max_blocking_time
    push_u32(rel, 0);
    push_param(p, ParameterId::PID_RELIABILITY, rel);
  }
  push_param(p, ParameterId::PID_UNICAST_LOCATOR, locator_value(7411));
  push_sentinel(p);
  return p;
}

// An RTPS message from `prefix` with one DATA submessage from `writer` to
// `reader`, sequence number `sn`, an optional inline QoS parameter list (with
// its sentinel) and an optional payload. The flags follow from what is there.
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
  push_u16(m, 16); // octetsToInlineQos: the two entity ids and the sequence number
  push_entity_id(m, reader);
  push_entity_id(m, writer);
  push_u32(m, 0);  // SequenceNumber_t.high
  push_u32(m, sn); // SequenceNumber_t.low
  m.insert(m.end(), inline_qos.begin(), inline_qos.end());
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}

// Inline QoS of a dispose: KEY_HASH (optional) and STATUS_INFO disposed.
std::vector<uint8_t> dispose_inline_qos(const rtps::Guid_t *key_hash) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> q;
  if (key_hash != nullptr) {
    push_param(q, ParameterId::PID_KEY_HASH, guid_value(*key_hash));
  }
  push_param(q, ParameterId::PID_STATUS_INFO, status_value(0x01));
  push_sentinel(q);
  return q;
}

void feed(rtps::Participant *part, const std::vector<uint8_t> &message) {
  part->newMessage(message.data(), static_cast<rtps::DataSize_t>(message.size()));
}

constexpr const char *kTopic = "churn_topic";
constexpr const char *kType = "ChurnType";

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");

  rtps::Reader *reader =
      domain.createReader(*part, kTopic, kType, /*reliable=*/false, {0, 0, 0, 0});
  CHECK(reader != nullptr, "createReader");
  rtps::Writer *writer = domain.createWriter(*part, kTopic, kType, /*reliable=*/false,
                                             /*enforceUnicast=*/false);
  CHECK(writer != nullptr, "createWriter");

  // The peer: discovered over SPDP, which also matches its SEDP writers to
  // our SEDP readers.
  const rtps::GuidPrefix_t peer{
      {kFastDdsVendor[0], kFastDdsVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, 1}};
  feed(part,
       data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                    rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, {}, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 1, "peer discovered");

  const rtps::Guid_t remote_writer{
      peer, {{0x00, 0x00, 0x01}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  uint32_t pub_sn = 0; // the peer's SEDP publications writer counts its DATAs
  const auto announce_writer = [&](const rtps::Guid_t &guid, const char *topic,
                                   const std::vector<uint8_t> &inline_qos) {
    feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                            rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn, inline_qos,
                            sedp_payload(guid, topic, kType, false)));
  };
  const auto dispose_writer = [&](const std::vector<uint8_t> &inline_qos,
                                  const std::vector<uint8_t> &payload) {
    feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                            rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn, inline_qos,
                            payload));
  };

  // 1. Announced: matched. The announcement's own inline QoS (a key hash, no
  //    status) does not make it a dispose.
  {
    using rtps::SMElement::ParameterId;
    std::vector<uint8_t> key_only;
    push_param(key_only, ParameterId::PID_KEY_HASH, guid_value(remote_writer));
    push_sentinel(key_only);
    announce_writer(remote_writer, kTopic, key_only);
  }
  CHECK(reader->getProxiesCount() == 1, "remote writer announced with a key hash matches");
  CHECK(reader->isProxy(remote_writer), "the matched proxy is the announced writer");

  // 2. Fast DDS dispose: KEY_HASH + STATUS_INFO inline, no payload.
  dispose_writer(dispose_inline_qos(&remote_writer), {});
  CHECK(reader->getProxiesCount() == 0, "Fast DDS form dispose unmatches the remote writer");

  // 3. Cyclone DDS dispose: STATUS_INFO inline, the GUID in a PL_CDR payload.
  announce_writer(remote_writer, kTopic, {});
  CHECK(reader->getProxiesCount() == 1, "re-announced: matched again");
  {
    using rtps::SMElement::ParameterId;
    std::vector<uint8_t> payload = encapsulation();
    push_param(payload, ParameterId::PID_ENDPOINT_GUID, guid_value(remote_writer));
    push_sentinel(payload);
    dispose_writer(dispose_inline_qos(nullptr), payload);
  }
  CHECK(reader->getProxiesCount() == 0, "Cyclone DDS form dispose unmatches the remote writer");

  // 4. This engine's own form: KEY_HASH + STATUS_INFO inline, a bare sentinel
  //    as payload (SEDPAgent::announceEndpointDeletion).
  announce_writer(remote_writer, kTopic, {});
  CHECK(reader->getProxiesCount() == 1, "re-announced once more: matched");
  dispose_writer(dispose_inline_qos(&remote_writer), sentinel());
  CHECK(reader->getProxiesCount() == 0, "espp form dispose unmatches the remote writer");

  // 5. An endpoint nobody matched waits in the registry of known remote endpoints, from where
  //    a reader created later still matches it - unless its dispose came
  //    first. A dispose for an unknown GUID changes nothing.
  const rtps::Guid_t kept_writer{
      peer, {{0x00, 0x00, 0x02}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  const rtps::Guid_t gone_writer{
      peer, {{0x00, 0x00, 0x03}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  announce_writer(kept_writer, "later_topic", {});
  announce_writer(gone_writer, "gone_topic", {});
  const rtps::Guid_t unknown{
      peer, {{0x00, 0x00, 0x7f}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  dispose_writer(dispose_inline_qos(&unknown), {});
  dispose_writer(dispose_inline_qos(&gone_writer), {});
  rtps::Reader *later = domain.createReader(*part, "later_topic", kType, false, {0, 0, 0, 0});
  rtps::Reader *gone = domain.createReader(*part, "gone_topic", kType, false, {0, 0, 0, 0});
  CHECK(later != nullptr && gone != nullptr, "createReader for the registry checks");
  CHECK(later->getProxiesCount() == 1 && later->isProxy(kept_writer),
        "a writer announced before its reader existed is matched from the registry");
  CHECK(gone->getProxiesCount() == 0,
        "a writer disposed while unmatched is cleared from the registry");

  // 6. The subscriptions leg: a remote reader matched to our writer.
  const rtps::Guid_t remote_reader{
      peer, {{0x00, 0x00, 0x04}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 1, {},
                          sedp_payload(remote_reader, kTopic, kType, false)));
  CHECK(writer->getProxiesCount() == 1, "remote reader announced: matched to the local writer");
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 2,
                          dispose_inline_qos(&remote_reader), {}));
  CHECK(writer->getProxiesCount() == 0, "the remote reader's dispose unmatches it");

  // 7. Only the owner may dispose. A second peer B announces a writer; a
  //    dispose from A naming B's writer, an SPDP dispose from A naming B's
  //    participant, and a dispose on the wrong SEDP leg all change nothing;
  //    B's own dispose does.
  const rtps::GuidPrefix_t peer_b{
      {kFastDdsVendor[0], kFastDdsVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, 2}};
  feed(part,
       data_message(peer_b, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                    rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, {}, spdp_payload(peer_b)));
  CHECK(part->getRemoteParticipantCount() == 2, "peer B discovered");
  const rtps::Guid_t writer_b{
      peer_b, {{0x00, 0x00, 0x01}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  feed(part, data_message(peer_b, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, 1, {},
                          sedp_payload(writer_b, kTopic, kType, false)));
  CHECK(reader->getProxiesCount() == 1 && reader->isProxy(writer_b), "B's writer matched");
  dispose_writer(dispose_inline_qos(&writer_b), {}); // from A
  CHECK(reader->isProxy(writer_b), "a dispose from A naming B's writer is ignored");
  const rtps::Guid_t participant_b{peer_b, rtps::ENTITYID_BUILD_IN_PARTICIPANT};
  feed(part, data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                          rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 2,
                          dispose_inline_qos(&participant_b), {}));
  CHECK(part->getRemoteParticipantCount() == 2 && reader->isProxy(writer_b),
        "an SPDP dispose from A naming B's participant is ignored");
  const rtps::Guid_t reader_a2{
      peer, {{0x00, 0x00, 0x05}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 3, {},
                          sedp_payload(reader_a2, kTopic, kType, false)));
  CHECK(writer->getProxiesCount() == 1, "A's second reader matched to the local writer");
  dispose_writer(dispose_inline_qos(&reader_a2), {}); // publications leg: wrong kind
  CHECK(writer->getProxiesCount() == 1, "a reader disposed over the publications leg is ignored");
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 4,
                          dispose_inline_qos(&reader_a2), {}));
  CHECK(writer->getProxiesCount() == 0, "... and over the subscriptions leg it is unmatched");
  feed(part, data_message(peer_b, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, 2,
                          dispose_inline_qos(&writer_b), {}));
  CHECK(reader->getProxiesCount() == 0, "B's own dispose unmatches its writer");

  // 8. The participant leaves: its SPDP dispose removes it and every proxy of
  //    its endpoints.
  announce_writer(remote_writer, kTopic, {});
  CHECK(reader->getProxiesCount() == 1, "matched before the participant leaves");
  const rtps::Guid_t participant_guid{peer, rtps::ENTITYID_BUILD_IN_PARTICIPANT};
  feed(part, data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                          rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 3,
                          dispose_inline_qos(&participant_guid), {}));
  CHECK(part->getRemoteParticipantCount() == 1, "the participant's dispose removes it (B stays)");
  CHECK(reader->getProxiesCount() == 0, "its endpoints' proxies go with it");
  CHECK(part->findRemoteParticipant(peer) == nullptr, "the participant is no longer known");

  // 8b. A delayed copy of A's announcement (SN 1, from before its dispose at
  //     SN 3) must not bring it back: the SPDP reader filters sequence numbers
  //     only for the writers it knows, and A's went with A. A later one does.
  feed(part,
       data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                    rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, {}, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 1,
        "an announcement from before the dispose does not bring the participant back");
  feed(part,
       data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                    rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 4, {}, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 2, "a later announcement does");
  feed(part, data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                          rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 5,
                          dispose_inline_qos(&participant_guid), {}));
  CHECK(part->getRemoteParticipantCount() == 1, "... and its next dispose removes it again");

  // 9a. A payload-form dispose from B whose GUID names one of B's endpoints,
  //     not B itself, is malformed and changes nothing.
  {
    using rtps::SMElement::ParameterId;
    std::vector<uint8_t> payload = encapsulation();
    push_param(payload, ParameterId::PID_PARTICIPANT_GUID, guid_value(writer_b));
    push_sentinel(payload);
    feed(part, data_message(peer_b, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                            rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 2,
                            dispose_inline_qos(nullptr), payload));
  }
  CHECK(part->getRemoteParticipantCount() == 1,
        "a payload-form dispose naming an endpoint rather than the participant is ignored");

  // 9. B leaves without a key hash, its participant GUID in a BIG-endian
  //    PL_CDR payload (the SPDP payload parser honours the encapsulation's
  //    byte order, as the announcement parser does).
  {
    using rtps::SMElement::ParameterId;
    std::vector<uint8_t> payload{0x00, 0x02, 0x00, 0x00}; // PL_CDR_BE, options
    const auto push_u16_be = [&payload](uint16_t v) {
      payload.push_back(static_cast<uint8_t>(v >> 8));
      payload.push_back(static_cast<uint8_t>(v & 0xFF));
    };
    push_u16_be(static_cast<uint16_t>(ParameterId::PID_PARTICIPANT_GUID));
    push_u16_be(16);
    push_guid(payload, participant_b);
    push_u16_be(static_cast<uint16_t>(ParameterId::PID_SENTINEL));
    push_u16_be(0);
    feed(part, data_message(peer_b, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                            rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 3,
                            dispose_inline_qos(nullptr), payload));
  }
  CHECK(part->getRemoteParticipantCount() == 0, "a big-endian payload-form participant dispose");

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
