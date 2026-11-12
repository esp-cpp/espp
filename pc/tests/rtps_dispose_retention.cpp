// A dispose-after-write change stays in the writer's history for its whole
// retention, however many are sent in a burst.
//
// An endpoint deletion is announced as a dispose on the SEDP writer and the
// change is kept for a few seconds in case a peer needs it resent
// (StatefulWriter::dropDisposeAfterWriteChanges). The numbers waiting for
// that retention sat in a queue of 10 regardless of the history's size, and
// when the queue was full the change just sent was dropped from history at
// once - more than ten endpoint deletions within the retention window, under
// loss, left a peer with a stale matched endpoint until the participant lease
// ran out. The queue is now sized like the history (so with static storage it
// cannot be the smaller of the two), and should it still fill (a host history
// that grew past the profile depth) the OLDEST retained dispose is given up,
// not the newest.
//
// A reliable writer with one matched remote reader (announced through a
// hand-crafted SEDP DATA; nothing listens at its locator, which is fine)
// publishes HISTORY_SIZE_STATEFUL dispose-after-write changes in a burst,
// twice:
//  1. Right after the burst is sent, every one of them is still in history.
//  2. A retention later (4 s), none is.
//
// Exits 0 on success, 1 on the first failed check.

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "rtps/entities/Domain.hpp"
#include "rtps/entities/Reader.hpp"
#include "rtps/entities/StatefulWriter.hpp"
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

void feed(rtps::Participant *part, const std::vector<uint8_t> &message) {
  part->newMessage(message.data(), static_cast<rtps::DataSize_t>(message.size()));
}

constexpr const char *kTopic = "retention_topic";
constexpr const char *kType = "RetentionType";

bool burst(rtps::StatefulWriter &writer, int round) {
  const auto n = static_cast<int>(rtps::Config::HISTORY_SIZE_STATEFUL);
  std::vector<rtps::SequenceNumber_t> sns;
  const uint8_t payload[8] = {0, 1, 0, 0, 1, 2, 3, 4};
  for (int i = 0; i < n; ++i) {
    const auto *change = writer.newChange(rtps::ChangeKind_t::ALIVE, payload, sizeof(payload),
                                          /*inLineQoS=*/false, /*markDisposedAfterWrite=*/true);
    CHECK(change != nullptr, "newChange (dispose-after-write)");
    sns.push_back(change->sequenceNumber);
  }
  // Let the transport's workers send the burst (one DATA per progress run).
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  for (const auto &sn : sns) {
    if (!writer.isInHistory(sn)) {
      std::printf("FAIL: round %d: SN %u dropped from history right after the burst\n", round,
                  sn.low);
      return false;
    }
  }
  // The retention is 4 s from each send; the protocol task sweeps on its
  // heartbeat cadence.
  std::this_thread::sleep_for(std::chrono::milliseconds(6500));
  for (const auto &sn : sns) {
    if (writer.isInHistory(sn)) {
      std::printf("FAIL: round %d: SN %u still in history after the retention\n", round, sn.low);
      return false;
    }
  }
  return true;
}

bool run_checks() {
  rtps::Domain domain(kIp);
  rtps::Participant *part = domain.createParticipant();
  CHECK(part != nullptr, "createParticipant");
  CHECK(domain.completeInit(), "completeInit");

  const rtps::GuidPrefix_t peer{{kVendor[0], kVendor[1], 0xaa, 0xbb, 0xcc, 0xdd, 0, 0, 0, 0, 0, 5}};
  feed(part, data_message(peer, rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER,
                          rtps::ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER, 1, spdp_payload(peer)));
  CHECK(part->getRemoteParticipantCount() == 1, "peer discovered");

  auto *writer = static_cast<rtps::StatefulWriter *>(
      domain.createWriter(*part, kTopic, kType, /*reliable=*/true, /*enforceUnicast=*/false));
  CHECK(writer != nullptr, "createWriter");
  const rtps::Guid_t remote_reader{
      peer, {{0x00, 0x00, 0x02}, rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY}};
  feed(part, data_message(peer, rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
                          rtps::ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER, 1,
                          sedp_payload(remote_reader, kTopic, kType, true)));
  CHECK(writer->getProxiesCount() == 1, "remote reader matched to the writer");

  const bool ok = burst(*writer, 1) && burst(*writer, 2);
  domain.stop();
  return ok;
}

} // namespace

int main() {
  if (!run_checks()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
