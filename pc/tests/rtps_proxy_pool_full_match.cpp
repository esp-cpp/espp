// A remote writer that finds no room in a reader's proxy pool is not lost.
//
// A reader holds NUM_WRITER_PROXIES_PER_READER remote writers (static storage:
// the embedded profile; host builds grow the pool instead). When the pool was
// full, SEDP's addNewMatchedWriter() failed and the agent dropped the writer
// on the floor: not logged through the logger, not kept in the registry of
// known remote endpoints, so it was never delivered until its participant was forgotten
// and rediscovered (and the eviction policy could count that participant as
// unused). Now the agent warns, naming the knob, keeps the writer in the
// registry, and retries the registry whenever a proxy is removed (a
// dispose, or a participant leaving), so the waiting writer takes the freed
// slot. The same for remote readers on local writers.
//
// Hand-crafted datagrams through Participant::newMessage; no network. The
// pool can only fill under static storage (RTPS_STORAGE_STATIC, the embedded
// profile; the interop QEMU firmware), so the checks are in two forms:
//  - static storage: NUM_WRITER_PROXIES_PER_READER + 1 remote writers on one
//    topic for one local reader match exactly NUM_WRITER_PROXIES_PER_READER,
//    the overflow writer is not among them, and it IS once one matched writer
//    is disposed (Fast DDS form); the same for NUM_READER_PROXIES_PER_WRITER +
//    1 remote readers on a local writer, freed by their participant leaving
//    (SPDP dispose) - the overflow reader belongs to a second participant and
//    must then be matched.
//  - dynamic storage (every host build, so the matrix runs this form): the
//    pools grow, every announced endpoint is matched at once, and a dispose /
//    a departure removes exactly its proxies. The static form is what the
//    device runs; this form pins that the same code path keeps the host
//    contract.
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

// A DATA with an optional inline QoS parameter list (with its sentinel).
std::vector<uint8_t> data_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                  const rtps::EntityId_t &reader, uint32_t sn,
                                  const std::vector<uint8_t> &inline_qos,
                                  const std::vector<uint8_t> &payload) {
  auto m = header(prefix);
  std::vector<uint8_t> b;
  push_u16(b, 0);
  push_u16(b, 16);
  push_entity_id(b, reader);
  push_entity_id(b, writer);
  push_u32(b, 0);
  push_u32(b, sn);
  b.insert(b.end(), inline_qos.begin(), inline_qos.end());
  b.insert(b.end(), payload.begin(), payload.end());
  uint8_t flags = rtps::FLAG_LITTLE_ENDIAN;
  if (!inline_qos.empty()) {
    flags |= rtps::FLAG_INLINE_QOS;
  }
  if (!payload.empty()) {
    flags |= rtps::FLAG_DATA_PAYLOAD;
  }
  push_submessage(m, static_cast<uint8_t>(rtps::SubmessageKind::DATA), flags, b);
  return m;
}

// Inline QoS of a dispose: KEY_HASH + STATUS_INFO disposed (Fast DDS form).
std::vector<uint8_t> dispose_inline_qos(const rtps::Guid_t &key_hash) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> q;
  push_param(q, ParameterId::PID_KEY_HASH, guid_value(key_hash));
  push_param(q, ParameterId::PID_STATUS_INFO, {0, 0, 0, 1});
  push_sentinel(q);
  return q;
}

void feed(rtps::Participant *part, const std::vector<uint8_t> &message) {
  part->newMessage(message.data(), static_cast<rtps::DataSize_t>(message.size()));
}

rtps::GuidPrefix_t peer_prefix(uint8_t last) {
  return rtps::GuidPrefix_t{{kVendor[0], kVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, last}};
}

rtps::Guid_t writer_guid(const rtps::GuidPrefix_t &peer, uint8_t key) {
  return rtps::Guid_t{peer,
                      {{0x00, 0x00, key}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
}

rtps::Guid_t reader_guid(const rtps::GuidPrefix_t &peer, uint8_t key) {
  return rtps::Guid_t{peer,
                      {{0x00, 0x00, key}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
}

constexpr const char *kType = "PoolType";

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");

  const rtps::GuidPrefix_t peer = peer_prefix(1);
  feed(part,
       data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                    rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, {}, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 1, "peer discovered");
  uint32_t pub_sn = 0;
  uint32_t sub_sn = 0;
#ifdef RTPS_STORAGE_DYNAMIC
  constexpr bool kPoolsGrow = true;
#else
  constexpr bool kPoolsGrow = false;
#endif

  // 1. One more remote writer than the reader's pool holds.
  rtps::Reader *reader = domain.createReader(*part, "pool_topic", kType, false, {0, 0, 0, 0});
  CHECK(reader != nullptr, "createReader");
  const auto pool = static_cast<unsigned>(rtps::Config::NUM_WRITER_PROXIES_PER_READER);
  for (unsigned i = 1; i <= pool + 1; ++i) {
    feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                            rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn, {},
                            sedp_payload(writer_guid(peer, static_cast<uint8_t>(i)), "pool_topic",
                                         kType, false)));
  }
  const rtps::Guid_t overflow = writer_guid(peer, static_cast<uint8_t>(pool + 1));
  CHECK(reader->isProxy(writer_guid(peer, 1)), "the first writer is matched");
  if (kPoolsGrow) {
    CHECK(reader->getProxiesCount() == pool + 1, "dynamic storage: the pool grew by one");
    CHECK(reader->isProxy(overflow), "dynamic storage: the writer past the cap is matched too");
  } else {
    CHECK(reader->getProxiesCount() == pool, "the reader holds a full pool of writers");
    CHECK(!reader->isProxy(overflow), "the writer past the pool is not matched");
  }

  // 2. One matched writer goes away: under static storage the waiting one
  //    takes its slot; under dynamic storage only the disposed one is gone.
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++pub_sn,
                          dispose_inline_qos(writer_guid(peer, 1)), {}));
  CHECK(!reader->isProxy(writer_guid(peer, 1)), "the disposed writer is unmatched");
  CHECK(reader->isProxy(overflow), "the writer that waited for a slot is matched now");
  CHECK(reader->getProxiesCount() == pool, "the pool holds exactly its capacity again");

  // 3. Remote readers on a local writer, the pool freed by a participant leaving.
  rtps::Writer *writer = domain.createWriter(*part, "pool_w_topic", kType, true, false);
  CHECK(writer != nullptr, "createWriter");
  const auto wpool = static_cast<unsigned>(rtps::Config::NUM_READER_PROXIES_PER_WRITER);
  for (unsigned i = 1; i <= wpool; ++i) {
    feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                            rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, ++sub_sn, {},
                            sedp_payload(reader_guid(peer, static_cast<uint8_t>(i)), "pool_w_topic",
                                         kType, false)));
  }
  const rtps::GuidPrefix_t other = peer_prefix(2);
  feed(part,
       data_message(other, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                    rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, {}, spdp_payload(other)));
  CHECK(part->getRemoteParticipantCount() == 2, "second peer discovered");
  const rtps::Guid_t late_reader = reader_guid(other, 1);
  feed(part, data_message(other, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 1, {},
                          sedp_payload(late_reader, "pool_w_topic", kType, false)));
  if (kPoolsGrow) {
    CHECK(writer->getProxiesCount() == wpool + 1, "dynamic storage: the writer's pool grew");
    CHECK(writer->isProxy(late_reader), "dynamic storage: the late reader is matched too");
  } else {
    CHECK(writer->getProxiesCount() == wpool, "the writer holds a full pool of readers");
    CHECK(!writer->isProxy(late_reader), "the reader past the pool is not matched");
  }
  // The first peer leaves: its readers' proxies go, the late reader (is or)
  // gets in, alone.
  feed(part, data_message(
                 peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                 rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 2,
                 dispose_inline_qos(rtps::Guid_t{peer, rtps::ENTITYID_BUILD_IN_PARTICIPANT}), {}));
  CHECK(part->getRemoteParticipantCount() == 1, "the first peer is gone");
  CHECK(writer->isProxy(late_reader), "the reader that waited for a slot is matched now");
  CHECK(writer->getProxiesCount() == 1, "only the late reader remains matched");
  CHECK(reader->getProxiesCount() == 0, "the departed peer's writers are all unmatched");
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
