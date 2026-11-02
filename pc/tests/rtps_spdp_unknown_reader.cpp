// SPDP DATA addressed to the participant (readerId ENTITYID_UNKNOWN) must reach
// the SPDP reader.
//
// RTPS 2.5 8.3.7.2 lets a DATA leave its reader unnamed; the receiver then
// resolves the reader through the writer it is matched to. That works for SEDP
// and user data, whose readers hold a proxy per remote writer, but the SPDP
// reader is best-effort and matches no remote writer, so such a DATA found no
// reader and was dropped. CycloneDDS names the SPDP reader only in its first
// announcement to a newly seen peer; every periodic re-announcement goes out
// with ENTITYID_UNKNOWN and sequence number 1 again. Discovery succeeded, the
// re-announcements never reached SPDPAgent, and the peer was removed with all
// its matched endpoints once its lease ran out (esp-cpp/espp#871).
//
// The checks hand crafted datagrams to a participant through its receive entry
// point (Participant::newMessage); no network, no second process:
//  1. An announcement naming the SPDP reader adds the remote participant (the
//     path that always worked; the control).
//  2. A re-announcement with readerId ENTITYID_UNKNOWN and the same sequence
//     number refreshes that participant's liveliness. Fails without the fix.
//  3. A first announcement with readerId ENTITYID_UNKNOWN discovers a new
//     participant, so a peer that never names the reader is found too.
//  4. A DATA with readerId ENTITYID_UNKNOWN from a writer that is not the SPDP
//     writer, and that nothing matches, is still dropped: the routing is
//     scoped to the SPDP writer.
//
// Exits 0 on success, 1 on the first failed check.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/messages/MessageTypes.hpp"

namespace {

constexpr rtps::Ip4AddressBytes kIp{127, 0, 0, 1};
// CycloneDDS's vendor id, so the bytes look like the peer that showed the defect.
constexpr std::array<uint8_t, 2> kCycloneVendor{0x01, 0x10};

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

// Little-endian writers: the message and the PL_CDR_LE payload are both LE.
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

// One parameter: pid, length, value. Callers pass values whose size is a
// multiple of 4, as the parameter list is 4-byte aligned.
void push_param(std::vector<uint8_t> &p, rtps::SMElement::ParameterId pid,
                const std::vector<uint8_t> &value) {
  push_u16(p, static_cast<uint16_t>(pid));
  push_u16(p, static_cast<uint16_t>(value.size()));
  p.insert(p.end(), value.begin(), value.end());
}

// A UDPv4 locator on kIp, as a parameter value: kind, port, 16-byte address.
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

// The payload a peer announces itself with: PL_CDR_LE encapsulation, then the
// parameters ParticipantProxyData reads (protocol version, vendor, GUID, one
// metatraffic and one default unicast locator on our network so the SEDP
// proxies can be matched, the lease, the builtin endpoint set), then the
// sentinel.
std::vector<uint8_t> spdp_payload(const rtps::GuidPrefix_t &prefix, int32_t lease_s) {
  using rtps::SMElement::ParameterId;
  std::vector<uint8_t> p;
  p.insert(p.end(), rtps::SMElement::SCHEME_PL_CDR_LE.begin(),
           rtps::SMElement::SCHEME_PL_CDR_LE.end());
  push_u16(p, 0); // encapsulation options

  push_param(p, ParameterId::PID_PROTOCOL_VERSION,
             {rtps::PROTOCOLVERSION.major, rtps::PROTOCOLVERSION.minor, 0, 0});
  push_param(p, ParameterId::PID_VENDORID, {kCycloneVendor[0], kCycloneVendor[1], 0, 0});
  {
    std::vector<uint8_t> guid(prefix.id.begin(), prefix.id.end());
    push_entity_id(guid, rtps::ENTITYID_BUILD_IN_PARTICIPANT);
    push_param(p, ParameterId::PID_PARTICIPANT_GUID, guid);
  }
  push_param(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, locator_value(7410));
  push_param(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, locator_value(7411));
  {
    std::vector<uint8_t> lease;
    push_u32(lease, static_cast<uint32_t>(lease_s)); // Duration_t: int32 seconds
    push_u32(lease, 0);                              // uint32 fraction
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
  push_param(p, ParameterId::PID_SENTINEL, {});
  return p;
}

// An RTPS message from the participant `prefix` carrying one DATA submessage
// from `writer` to `reader` with sequence number `sn` and `payload`.
std::vector<uint8_t> data_message(const rtps::GuidPrefix_t &prefix, const rtps::EntityId_t &writer,
                                  const rtps::EntityId_t &reader, uint32_t sn,
                                  const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> m{'R',
                         'T',
                         'P',
                         'S',
                         rtps::PROTOCOLVERSION.major,
                         rtps::PROTOCOLVERSION.minor,
                         kCycloneVendor[0],
                         kCycloneVendor[1]};
  m.insert(m.end(), prefix.id.begin(), prefix.id.end());

  // DATA: header, extraFlags, octetsToInlineQos, readerId, writerId, writerSN, payload.
  m.push_back(static_cast<uint8_t>(rtps::SubmessageKind::DATA));
  m.push_back(static_cast<uint8_t>(rtps::FLAG_LITTLE_ENDIAN | rtps::FLAG_DATA_PAYLOAD));
  push_u16(m, static_cast<uint16_t>(2 + 2 + 4 + 4 + 8 + payload.size())); // octetsToNextHeader
  push_u16(m, 0);                                                         // extraFlags
  push_u16(m, 16); // octetsToInlineQos: the two entity ids and the sequence number
  push_entity_id(m, reader);
  push_entity_id(m, writer);
  push_u32(m, 0);  // SequenceNumber_t.high
  push_u32(m, sn); // SequenceNumber_t.low
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}

rtps::GuidPrefix_t peer_prefix(uint8_t last) {
  return rtps::GuidPrefix_t{
      {kCycloneVendor[0], kCycloneVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, last}};
}

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");
  CHECK(part->getRemoteParticipantCount() == 0, "no remote participants at start");

  // 1. The first announcement names the SPDP reader, as CycloneDDS's does.
  const rtps::GuidPrefix_t peer = peer_prefix(1);
  const auto payload = spdp_payload(peer, 10);
  const auto named = data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                                  rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, payload);
  part->newMessage(named.data(), static_cast<rtps::DataSize_t>(named.size()));
  const rtps::ParticipantProxyData *proxy = part->findRemoteParticipant(peer);
  CHECK(proxy != nullptr, "announcement naming the SPDP reader adds the participant");
  CHECK(part->getRemoteParticipantCount() == 1, "one remote participant");
  const auto first_seen = proxy->m_lastLivelinessReceivedTimestamp;

  // 2. The re-announcement leaves the reader unnamed and repeats the sequence
  //    number. It must refresh the lease, or the peer expires.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const auto repeat = data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                                   rtps::ENTITYID_UNKNOWN, 1, payload);
  part->newMessage(repeat.data(), static_cast<rtps::DataSize_t>(repeat.size()));
  proxy = part->findRemoteParticipant(peer);
  CHECK(proxy != nullptr, "participant still known after the re-announcement");
  CHECK(proxy->m_lastLivelinessReceivedTimestamp > first_seen,
        "re-announcement with readerId ENTITYID_UNKNOWN refreshes liveliness");
  CHECK(part->getRemoteParticipantCount() == 1, "re-announcement adds no duplicate");

  // 3. A peer whose very first announcement leaves the reader unnamed.
  const rtps::GuidPrefix_t other = peer_prefix(2);
  const auto first_unnamed = data_message(other, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                                          rtps::ENTITYID_UNKNOWN, 1, spdp_payload(other, 10));
  part->newMessage(first_unnamed.data(), static_cast<rtps::DataSize_t>(first_unnamed.size()));
  CHECK(part->findRemoteParticipant(other) != nullptr,
        "first announcement with readerId ENTITYID_UNKNOWN discovers the participant");
  CHECK(part->getRemoteParticipantCount() == 2, "two remote participants");

  // 4. Unnamed reader, but not the SPDP writer: the matched-writer lookup finds
  //    nothing and the DATA is dropped, as before.
  const rtps::GuidPrefix_t stray = peer_prefix(3);
  const rtps::EntityId_t user_writer{{0x00, 0x00, 0x01},
                                     rtps::EntityKind_t::USER_DEFINED_WRITER_WITH_KEY};
  const auto not_spdp =
      data_message(stray, user_writer, rtps::ENTITYID_UNKNOWN, 1, spdp_payload(stray, 10));
  part->newMessage(not_spdp.data(), static_cast<rtps::DataSize_t>(not_spdp.size()));
  CHECK(part->findRemoteParticipant(stray) == nullptr,
        "an unnamed-reader DATA from a non-SPDP writer is not routed to the SPDP reader");
  CHECK(part->getRemoteParticipantCount() == 2, "still two remote participants");

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
