// Golden wire-format tests for the embeddedRTPS engine (components/rtps_embedded).
//
// Phase 0b of components/rtps_embedded/REFACTOR_PLAN.md: freeze the engine's current
// (FastDDS/ROS2-interop-proven, Micro-CDR-based) message encodings byte-for-byte, so
// codec changes (e.g. the Micro-CDR removal) and refactors can be verified to be
// wire-neutral. Covers the RTPS header, INFO_DST, INFO_TS(invalid), DATA, HEARTBEAT,
// ACKNACK (multi-word MSB-first SequenceNumberSet), GAP, and the SEDP TopicData
// PL_CDR parameter list (string alignment, PID_SENTINEL, locators) incl. round-trip.
//
// Regenerate goldens (only when an intentional wire change is made):
//   ./rtps_embedded_golden --dump > ../tests/rtps_embedded_golden.inc
//
// Exits 0 when every section matches its golden byte string; 1 otherwise.

#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "rtps/discovery/TopicData.hpp"
#include "rtps/messages/MessageFactory.hpp"
#include "rtps/storages/PayloadBuffer.hpp"
#include "rtps/utils/CdrBuffer.hpp"

namespace {

struct Golden {
  const char *name;
  std::span<const uint8_t> bytes;
};

// Fixed inputs shared by all sections
constexpr rtps::GuidPrefix_t kPrefix{
    {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C}};
const rtps::EntityId_t kWriterId{{0x00, 0x00, 0x01},
                                 rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY};
const rtps::EntityId_t kReaderId{{0x00, 0x00, 0x02},
                                 rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY};

std::vector<uint8_t> build_header() {
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addHeader(b, kPrefix);
  return b.bytes;
}

std::vector<uint8_t> build_info_dst() {
  rtps::PayloadBuffer b;
  rtps::GuidPrefix_t dst = kPrefix;
  rtps::MessageFactory::addSubMessageInfoDST(b, dst);
  return b.bytes;
}

std::vector<uint8_t> build_info_ts_invalid() {
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addSubMessageTimeStamp(b, /*setInvalid=*/true);
  return b.bytes;
}

std::vector<uint8_t> build_data() {
  // CDR_LE-encapsulated "hello" string payload (4B encap + 4B length + 6B chars)
  static constexpr uint8_t kPayload[] = {0x00, 0x01, 0x00, 0x00, 0x06, 0x00, 0x00,
                                         0x00, 'h',  'e',  'l',  'l',  'o',  0x00};
  rtps::PayloadBuffer payload;
  payload.append(kPayload, sizeof(kPayload));
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addSubMessageData(b, payload, /*containsInlineQos=*/false,
                                          rtps::SequenceNumber_t{0, 5}, kWriterId, kReaderId);
  return b.bytes;
}

std::vector<uint8_t> build_data_related_sample_identity() {
  // DATA submessage carrying a related_sample_identity inline QoS (ROS 2 service
  // request/reply correlation). Pins the inline-QoS layout: PID 0x0083 + 0x800f,
  // each a 24-byte SampleIdentity {Guid, SequenceNumber}, then PID_SENTINEL,
  // followed by the payload. The identity's guid uses kPrefix + a reader entity
  // and sequence number {0, 1}, matching the shape seen in the live capture.
  static constexpr uint8_t kPayload[] = {0x00, 0x01, 0x00, 0x00, 0x65, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  rtps::PayloadBuffer payload;
  payload.append(kPayload, sizeof(kPayload));
  rtps::rpc::SampleIdentity related{rtps::Guid_t{kPrefix, kReaderId}, rtps::SequenceNumber_t{0, 1}};
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addSubMessageDataWithRelatedSampleIdentity(
      b, payload, related, rtps::SequenceNumber_t{0, 1}, kWriterId, kReaderId);
  return b.bytes;
}

std::vector<uint8_t> build_data_frag() {
  // One DATA_FRAG submessage: fragment #1 of a 200000-byte sample split at a
  // 63000-byte fragment size, carrying a 10-byte ramp payload chunk. Pins the
  // DATA_FRAG wire layout (extraFlags, octetsToInlineQos, ids, writerSN,
  // fragmentStartingNum, fragmentsInSubmessage, fragmentSize, sampleSize, data).
  static constexpr uint8_t kFrag[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addSubMessageDataFrag(b, kFrag, sizeof(kFrag), /*fragStartNum=*/1,
                                              /*fragsInSubmsg=*/1, /*fragmentSize=*/63000,
                                              /*sampleSize=*/200000, rtps::SequenceNumber_t{0, 5},
                                              kWriterId, kReaderId);
  return b.bytes;
}

std::vector<uint8_t> build_heartbeat() {
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addHeartbeat(b, kWriterId, kReaderId, rtps::SequenceNumber_t{0, 1},
                                     rtps::SequenceNumber_t{0, 7}, rtps::Count_t{3});
  return b.bytes;
}

std::vector<uint8_t> build_acknack() {
  // Multi-word SequenceNumberSet: 40 bits, MSB-first bit order
  // (bitMap[bucket] & 1 << (31 - pos)): bits 0 and 31 in word 0, bit 33 in word 1.
  rtps::SequenceNumberSet sns;
  sns.base = rtps::SequenceNumber_t{0, 4};
  sns.numBits = 40;
  sns.bitMap[0] = 0x80000001;
  sns.bitMap[1] = 0x40000000;
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addAckNack(b, kWriterId, kReaderId, sns, rtps::Count_t{9},
                                   /*final_flag=*/false);
  return b.bytes;
}

std::vector<uint8_t> build_gap() {
  rtps::PayloadBuffer b;
  rtps::MessageFactory::addSubmessageGap(b, kWriterId, kReaderId, rtps::SequenceNumber_t{0, 5},
                                         rtps::SequenceNumber_t{0, 9});
  return b.bytes;
}

rtps::TopicData make_topic_data() {
  rtps::TopicData td;
  td.endpointGuid.prefix = kPrefix;
  td.endpointGuid.entityId = kWriterId;
  // Odd-length names exercise the CDR string length-prefix + null terminator +
  // 4-byte parameter alignment corner cases.
  std::strncpy(td.typeName, "std_msgs::msg::dds_::String_", sizeof(td.typeName));
  std::strncpy(td.topicName, "rt/chatter", sizeof(td.topicName));
  td.reliabilityKind = rtps::ReliabilityKind_t::RELIABLE;
  td.durabilityKind = rtps::DurabilityKind_t::TRANSIENT_LOCAL;
  td.unicastLocator = rtps::FullLengthLocator::createUDPv4Locator(192, 168, 1, 2, 7411);
  td.multicastLocator = rtps::FullLengthLocator::createUDPv4Locator(239, 255, 0, 1, 7400);
  return td;
}

std::vector<uint8_t> build_sedp_topic_data() {
  const rtps::TopicData td = make_topic_data();
  std::vector<uint8_t> buf(1024, 0);
  rtps::CdrSink sink{rtps::asWritableBytes(buf.data(), buf.size())};
  rtps::CdrWriter writer(sink);
  if (!td.serializeInto(writer)) {
    return {};
  }
  buf.resize(sink.size());
  return buf;
}

bool roundtrip_sedp_topic_data() {
  const rtps::TopicData td = make_topic_data();
  std::vector<uint8_t> bytes = build_sedp_topic_data();
  if (bytes.empty()) {
    std::printf("FAIL: sedp serialize returned no bytes\n");
    return false;
  }
  rtps::TopicData parsed;
  if (!parsed.readFromBuffer(std::span<const uint8_t>(bytes.data(), bytes.size()))) {
    std::printf("FAIL: sedp round-trip parse failed\n");
    return false;
  }
  if (std::strcmp(parsed.topicName, td.topicName) != 0 ||
      std::strcmp(parsed.typeName, td.typeName) != 0 || !(parsed.endpointGuid == td.endpointGuid) ||
      parsed.reliabilityKind != td.reliabilityKind) {
    std::printf("FAIL: sedp round-trip field mismatch\n");
    return false;
  }
  return true;
}

void dump_array(const char *name, const std::vector<uint8_t> &bytes) {
  std::printf("static constexpr uint8_t kGolden_%s[] = {", name);
  for (size_t i = 0; i < bytes.size(); i++) {
    if (i % 12 == 0) {
      std::printf("\n    ");
    }
    std::printf("0x%02X,%s", bytes[i], (i + 1 < bytes.size()) ? " " : "");
  }
  std::printf("};\n");
}

bool check(const char *name, const std::vector<uint8_t> &actual, std::span<const uint8_t> golden) {
  if (actual.size() == golden.size() &&
      std::memcmp(actual.data(), golden.data(), golden.size()) == 0) {
    std::printf("PASS: %-16s (%zu bytes)\n", name, actual.size());
    return true;
  }
  std::printf("FAIL: %-16s actual %zu bytes vs golden %zu bytes\n", name, actual.size(),
              golden.size());
  const size_t n = std::min(actual.size(), golden.size());
  for (size_t i = 0; i < n; i++) {
    if (actual[i] != golden[i]) {
      std::printf("  first diff at byte %zu: actual 0x%02X vs golden 0x%02X\n", i, actual[i],
                  golden[i]);
      break;
    }
  }
  return false;
}

// Golden byte strings captured from the current (interop-proven) implementation.
#include "rtps_embedded_golden.inc"

} // namespace

int main(int argc, char **argv) {
  struct Section {
    const char *name;
    std::vector<uint8_t> (*build)();
    std::span<const uint8_t> golden;
  };
  const Section sections[] = {
      {"header", build_header, kGolden_header},
      {"info_dst", build_info_dst, kGolden_info_dst},
      {"info_ts_invalid", build_info_ts_invalid, kGolden_info_ts_invalid},
      {"data", build_data, kGolden_data},
      {"data_related_sample_identity", build_data_related_sample_identity,
       kGolden_data_related_sample_identity},
      {"data_frag", build_data_frag, kGolden_data_frag},
      {"heartbeat", build_heartbeat, kGolden_heartbeat},
      {"acknack", build_acknack, kGolden_acknack},
      {"gap", build_gap, kGolden_gap},
      {"sedp_topic_data", build_sedp_topic_data, kGolden_sedp_topic_data},
  };

  if (argc > 1 && std::string(argv[1]) == "--dump") {
    std::printf("// Golden wire-format byte strings for rtps_embedded_golden.cpp.\n");
    std::printf("// Generated by: ./rtps_embedded_golden --dump > "
                "../tests/rtps_embedded_golden.inc\n");
    std::printf("// Do NOT edit by hand; regenerate only on an intentional wire change.\n");
    for (const auto &s : sections) {
      dump_array(s.name, s.build());
    }
    return 0;
  }

  bool ok = true;
  for (const auto &s : sections) {
    ok &= check(s.name, s.build(), s.golden);
  }
  ok &= roundtrip_sedp_topic_data();
  std::printf(ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
