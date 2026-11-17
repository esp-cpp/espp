// A peer that restarts is rediscovered at once, not when its lease runs out.
//
// The GUID prefix of an embedded participant is built from the board's MAC,
// so a board that rebooted within its lease used to come back with the prefix
// its peers still held - and every one of its writers numbering its samples
// from 1 again. Its announcements only refreshed the known participant's
// lease, the stateful readers kept waiting for the old numbers and a
// best-effort reader ignored the restarted writer (RTPS 2.5 8.4.12.1), so
// nothing moved until the lease sweep forgot the old incarnation (5 s for an
// ESP peer, 100 s for a host one). This test drives the receiving side of the
// fix with hand-crafted datagrams fed to a real participant, as the dispose
// tests do:
//
//  1. Restart detection: an announcement from a known participant numbered
//     far below its highest (SPDPAgent::isRestart) forgets the participant
//     and rediscovers it; its writers get fresh proxies, and a best-effort
//     reader receives the restarted writer's samples from 1 again.
//  2. A reordered or duplicated announcement (last - 1) is NOT a restart:
//     nothing is forgotten, a sample numbered after the ones delivered still
//     arrives.
//  3. Incarnation: a rebooted board of this engine comes back with new
//     incarnation bytes (10-11) and first disposes its previous incarnation
//     AS that incarnation (old prefix, its SPDP writer, a sequence number
//     above anything it sent): the old participant goes at once, the new one
//     is discovered, and a stale announcement of the old one still in flight
//     is ignored. A dispose naming a participant from any other prefix is
//     refused, as before.
//
// Exits 0 on success, 1 on the first failed check.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/Writer.hpp"
#include "rtps/messages/MessageTypes.hpp"

namespace {

constexpr rtps::Ip4AddressBytes kIp{127, 0, 0, 1};
const std::array<uint8_t, 2> kVendor = rtps::Config::VENDOR_ID.vendorId; // an espp-shaped peer

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

// A DATA submessage in one RTPS message from `prefix`, with optional inline
// QoS and payload.
std::vector<uint8_t> data_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                  const rtps::EntityId_t &reader, uint32_t sn,
                                  const std::vector<uint8_t> &inline_qos,
                                  const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> m{
      'R',        'T',       'P', 'S', rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor,
      kVendor[0], kVendor[1]};
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

// Inline QoS of a participant dispose: the key hash of the disposed
// participant and STATUS_INFO disposed|unregistered, as Fast DDS writes it and
// as this engine's SPDPAgent::sendPreviousIncarnationDispose does.
std::vector<uint8_t> participant_dispose_qos(const rtps::GuidPrefix_t &disposed) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> q;
  push_param(q, ParameterId::PID_KEY_HASH,
             guid_value(rtps::Guid_t{disposed, rtps::ENTITYID_BUILD_IN_PARTICIPANT}));
  push_param(q, ParameterId::PID_STATUS_INFO, {0, 0, 0, 3});
  push_sentinel(q);
  return q;
}

// A user sample: CDR_LE encapsulation + 4 bytes of payload.
std::vector<uint8_t> sample_payload(uint32_t value) {
  std::vector<uint8_t> p{0x00, 0x01, 0x00, 0x00};
  push_u32(p, value);
  return p;
}

void feed(rtps::Participant *part, const std::vector<uint8_t> &message) {
  part->newMessage(message.data(), static_cast<rtps::DataSize_t>(message.size()));
}

constexpr const char *kTopic = "restart_topic";
constexpr const char *kType = "RestartType";

std::atomic<int> g_received{0};
std::atomic<uint32_t> g_last_value{0};

void on_sample(void *, const rtps::ReaderCacheChange &change) {
  uint8_t buf[8] = {0};
  if (change.copyInto(buf, sizeof(buf))) {
    uint32_t v = 0;
    std::memcpy(&v, buf + 4, sizeof(v));
    g_last_value = v;
  }
  ++g_received;
}

// A simulated peer: its SPDP writer, its SEDP publications writer and one
// user writer, each numbering its DATAs independently - like a real stack.
struct Peer {
  rtps::GuidPrefix_t prefix;
  uint32_t spdp_sn = 0;
  uint32_t sedp_sn = 0;
  uint32_t data_sn = 0;
  rtps::Guid_t writer_guid() const {
    return {prefix, {{0x00, 0x00, 0x01}, rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY}};
  }
  void announce(rtps::Participant *part) {
    feed(part, data_message(prefix, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                            rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, ++spdp_sn, {},
                            spdp_payload(prefix)));
  }
  // An announcement that repeats an earlier number (a duplicated or reordered datagram).
  void announce_with(rtps::Participant *part, uint32_t sn) const {
    feed(part, data_message(prefix, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                            rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, sn, {},
                            spdp_payload(prefix)));
  }
  void announce_writer(rtps::Participant *part) {
    feed(part, data_message(prefix, rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
                            rtps::ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER, ++sedp_sn, {},
                            sedp_payload(writer_guid(), kTopic, kType, false)));
  }
  void publish(rtps::Participant *part, uint32_t value) {
    feed(part, data_message(prefix, writer_guid().entityId, rtps::ENTITYID_UNKNOWN, ++data_sn, {},
                            sample_payload(value)));
  }
  // Dispose `disposed` from this peer's SPDP writer, numbered `sn`.
  void dispose(rtps::Participant *part, const rtps::GuidPrefix_t &disposed, uint32_t sn) {
    feed(part, data_message(prefix, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                            rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, sn,
                            participant_dispose_qos(disposed), {}));
  }
  // The peer rebooted: every writer starts at 1 again.
  void reboot() { spdp_sn = sedp_sn = data_sn = 0; }
};

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");
  rtps::Reader *reader =
      domain.createReader(*part, kTopic, kType, /*reliable=*/false, {0, 0, 0, 0});
  CHECK(reader != nullptr, "createReader");
  reader->registerCallback(on_sample, nullptr);

  // An ESP-style prefix: MAC, participant id, this engine's vendor id, domain,
  // incarnation (the layout Domain::generateGuidPrefix produces on a board).
  const auto vid = rtps::Config::VENDOR_ID.vendorId;
  Peer peer{{{0x24, 0x6f, 0x28, 0xaa, 0xbb, 0xcc, 0, vid[0], vid[1], 0, 0x11, 0x22}}};

  // --- discovered, matched, delivering -------------------------------------------------
  for (int i = 0; i < 6; ++i) {
    peer.announce(part); // SPDP 1..6: a peer that has been around a while
  }
  CHECK(part->getRemoteParticipantCount() == 1, "peer discovered");
  peer.announce_writer(part);
  CHECK(reader->getProxiesCount() == 1, "peer writer matched");
  for (uint32_t v = 1; v <= 3; ++v) {
    peer.publish(part, v); // DATA 1..3
  }
  CHECK(g_received == 3, "three samples delivered");
  CHECK(g_last_value == 3, "the last one is sample 3");

  // --- 2. a duplicated / reordered announcement is not a restart --------------------------
  peer.announce_with(part, 5); // 5 after 6: one datagram late
  CHECK(part->getRemoteParticipantCount() == 1, "late announcement: participant kept");
  CHECK(reader->getProxiesCount() == 1, "late announcement: writer proxy kept");
  peer.publish(part, 4); // DATA 4 continues the stream
  CHECK(g_received == 4, "stream continues after a late announcement");
  peer.announce_with(part, 3); // half of 6: this one IS below the restart threshold
  // ... and that is a restart by the rule (3 * 2 <= 6); the peer is rediscovered
  // from this announcement, so its writer must be announced again - a real
  // restarted peer does exactly that. Check the proxy went, then re-announce.
  CHECK(part->getRemoteParticipantCount() == 1, "restart threshold: rediscovered participant");
  CHECK(reader->getProxiesCount() == 0, "restart threshold: the old writer proxy is gone");
  peer.sedp_sn = 0;
  peer.data_sn = 0;
  peer.announce_writer(part);
  CHECK(reader->getProxiesCount() == 1, "restart threshold: writer matched again");
  peer.publish(part, 10);
  CHECK(g_received == 5 && g_last_value == 10, "restart threshold: sample 1 of the new stream");
  // bring the peer's number up again for the next case
  peer.spdp_sn = 3;
  for (int i = 0; i < 6; ++i) {
    peer.announce(part); // 4..9
  }
  for (uint32_t v = 11; v <= 13; ++v) {
    peer.publish(part, v); // DATA 2..4
  }
  CHECK(g_received == 8 && g_last_value == 13, "stream continues: samples 2..4");

  // --- 1. the peer reboots with the same prefix, no dispose ---------------------------------
  peer.reboot();
  peer.announce(part); // SPDP 1 after 9
  CHECK(part->getRemoteParticipantCount() == 1, "rebooted peer: still exactly one participant");
  CHECK(reader->getProxiesCount() == 0, "rebooted peer: the old writer proxy is gone");
  peer.announce_writer(part); // SEDP 1
  CHECK(reader->getProxiesCount() == 1, "rebooted peer: its writer matched again");
  for (uint32_t v = 21; v <= 23; ++v) {
    peer.publish(part, v); // DATA 1..3 again - these used to be dropped
  }
  CHECK(g_received == 11, "rebooted peer: its samples 1..3 delivered");
  CHECK(g_last_value == 23, "rebooted peer: the last one is sample 3 of the new stream");

  // --- 3. the board reboots into a new incarnation and disposes the old one -----------------
  // What a rebooted board of this engine sends first (SPDPAgent::
  // sendPreviousIncarnationDispose): a dispose of the OLD participant, as the
  // old participant - its prefix, its SPDP writer - with a sequence number
  // above anything that writer reached. Then the new incarnation announces.
  Peer next = peer;
  next.reboot();
  next.prefix.id[10] = 0x33; // the incarnation bytes change, nothing else
  next.prefix.id[11] = 0x44;
  // a dispose from an unrelated prefix naming the peer: refused, as always
  Peer stranger{{{0x24, 0x6f, 0x28, 0x01, 0x02, 0x03, 0, vid[0], vid[1], 0, 0x11, 0x22}}};
  stranger.dispose(part, peer.prefix, 1);
  CHECK(part->getRemoteParticipantCount() == 1, "dispose from a stranger: refused");
  // ... and from the next incarnation's own writer naming the old one: another
  // prefix is another participant, refused too (the engine does not send that)
  next.dispose(part, peer.prefix, 1);
  CHECK(part->getRemoteParticipantCount() == 1, "dispose from another prefix: refused");
  CHECK(reader->getProxiesCount() == 1, "refused disposes change nothing");
  // the old incarnation's own dispose, numbered beyond its last announcement
  peer.dispose(part, peer.prefix, std::numeric_limits<uint32_t>::max());
  CHECK(part->getRemoteParticipantCount() == 0, "the previous incarnation's dispose: honoured");
  CHECK(reader->getProxiesCount() == 0, "previous incarnation: the old writer proxy is gone");
  next.announce(part);
  CHECK(part->getRemoteParticipantCount() == 1, "next incarnation discovered");
  next.announce_writer(part);
  CHECK(reader->getProxiesCount() == 1, "next incarnation: its writer matched");
  for (uint32_t v = 31; v <= 33; ++v) {
    next.publish(part, v);
  }
  CHECK(g_received == 14 && g_last_value == 33, "next incarnation: samples delivered");
  // a stale announcement of the old incarnation still in flight (a delayed
  // datagram, numbered below the dispose) must not bring the dead participant
  // back next to the new one
  peer.announce_with(part, 9);
  CHECK(part->getRemoteParticipantCount() == 1, "stale old-incarnation announcement: ignored");
  CHECK(reader->getProxiesCount() == 1, "stale old-incarnation announcement: new proxy kept");
  next.publish(part, 34);
  CHECK(g_received == 15 && g_last_value == 34, "new incarnation keeps delivering");

  domain.stop();
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
