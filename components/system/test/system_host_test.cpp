// Host-buildable unit tests for the SystemService wire codec
// (include/detail/system_protocol.hpp). Build & run with:
//   c++ -std=c++20 -I../include -I../../stream_frame/include system_host_test.cpp -o test && ./test
//
// The codec needs no ESP-IDF headers; these are golden encode/decode tests so
// the browser console and any host tool can rely on the byte layout.

#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include "detail/system_protocol.hpp"

namespace sp = espp::detail::system_protocol;
namespace sf = espp::stream_frame;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static void test_info_golden() {
  std::printf("test_info_golden\n");
  sp::InfoBuilder b;
  b.str(sp::InfoTag::ChipModel, "ESP32-S3")
      .u16(sp::InfoTag::ChipRevision, 2)
      .u8(sp::InfoTag::Cores, 2);
  const auto &p = b.payload();
  // [1][8]"ESP32-S3" [2][2][0x02 0x00] [3][1][2]
  const uint8_t golden[] = {1,   8, 'E', 'S',  'P',  '3', '2', '-', 'S',
                            '3', 2, 2,   0x02, 0x00, 3,   1,   2};
  CHECK(p.size() == sizeof(golden) && std::equal(golden, golden + sizeof(golden), p.begin()));
  const auto info = sp::decode_info(p);
  CHECK(info && info->chip_model == "ESP32-S3" && info->chip_revision == 2 && info->cores == 2);
  CHECK(info && !info->idf_version && info->unknown_tags == 0);
}

static void test_info_all_tags() {
  std::printf("test_info_all_tags\n");
  std::array<uint8_t, 32> sha{};
  for (size_t i = 0; i < sha.size(); ++i)
    sha[i] = static_cast<uint8_t>(i);
  const std::array<uint8_t, 6> mac = {0x24, 0x6F, 0x28, 0x01, 0x02, 0x03};
  sp::InfoBuilder b;
  b.str(sp::InfoTag::ChipModel, "ESP32-P4")
      .u16(sp::InfoTag::ChipRevision, 100)
      .u8(sp::InfoTag::Cores, 2)
      .u32(sp::InfoTag::ChipFeatures, 0x81)
      .str(sp::InfoTag::IdfVersion, "v6.1")
      .str(sp::InfoTag::ProjectName, "system_example")
      .str(sp::InfoTag::AppVersion, "1.2.3")
      .str(sp::InfoTag::BuildDate, "Sep 30 2026")
      .str(sp::InfoTag::BuildTime, "12:34:56")
      .bytes(sp::InfoTag::ElfSha256, sha)
      .str(sp::InfoTag::RunningPartition, "ota_0")
      .str(sp::InfoTag::BootPartition, "ota_1")
      .u8(sp::InfoTag::OtaState, 2)
      .u8(sp::InfoTag::ResetReason, 3)
      .u64(sp::InfoTag::UptimeMs, 0x0000000123456789ULL)
      .bytes(sp::InfoTag::Mac, mac)
      .u32(sp::InfoTag::FlashSize, 16 * 1024 * 1024)
      .u32(sp::InfoTag::PsramSize, 8 * 1024 * 1024)
      .u32(sp::InfoTag::CpuMhz, 360)
      .u32(sp::InfoTag::FreeHeap, 123456)
      .u32(sp::InfoTag::MinFreeHeap, 100000)
      .u32(sp::InfoTag::Capabilities, sp::kCapReboot | sp::kCapBootloader);
  const auto info = sp::decode_info(b.payload());
  CHECK(info);
  if (!info)
    return;
  CHECK(info->chip_model == "ESP32-P4" && info->chip_revision == 100 && info->cores == 2 &&
        info->chip_features == 0x81u);
  CHECK(info->idf_version == "v6.1" && info->project_name == "system_example" &&
        info->app_version == "1.2.3" && info->build_date == "Sep 30 2026" &&
        info->build_time == "12:34:56");
  CHECK(info->elf_sha256 && (*info->elf_sha256)[31] == 31);
  CHECK(info->running_partition == "ota_0" && info->boot_partition == "ota_1" &&
        info->ota_state == 2 && info->reset_reason == 3);
  CHECK(info->uptime_ms == 0x0000000123456789ULL);
  CHECK(info->mac && (*info->mac)[0] == 0x24 && (*info->mac)[5] == 0x03);
  CHECK(info->flash_size == 16u * 1024 * 1024 && info->psram_size == 8u * 1024 * 1024 &&
        info->cpu_mhz == 360);
  CHECK(info->free_heap == 123456 && info->min_free_heap == 100000);
  CHECK(info->capabilities == (sp::kCapReboot | sp::kCapBootloader));
  CHECK(info->unknown_tags == 0);
  // the u64 is little-endian on the wire: find the record and check its bytes
  const auto &p = b.payload();
  bool found = false;
  for (size_t i = 0; i + 1 < p.size();) {
    const uint8_t tag = p[i], len = p[i + 1];
    if (tag == static_cast<uint8_t>(sp::InfoTag::UptimeMs)) {
      CHECK(len == 8 && p[i + 2] == 0x89 && p[i + 3] == 0x67 && p[i + 5] == 0x23 && p[i + 9] == 0);
      found = true;
    }
    i += 2 + len;
  }
  CHECK(found);
}

static void test_info_unknown_and_truncated() {
  std::printf("test_info_unknown_and_truncated\n");
  // an unknown tag (200) with a 3-byte value is skipped; the record after it decodes
  std::vector<uint8_t> p = {200, 3, 0xAA, 0xBB, 0xCC, 3, 1, 1};
  auto info = sp::decode_info(p);
  CHECK(info && info->cores == 1 && info->unknown_tags == 1);
  // a known fixed-size tag with the wrong length is skipped, not misdecoded
  p = {3, 2, 1, 1, 19, 4, 0x68, 0x01, 0, 0};
  info = sp::decode_info(p);
  CHECK(info && !info->cores && info->cpu_mhz == 360 && info->unknown_tags == 1);
  // a record whose length runs past the payload is rejected
  p = {1, 8, 'E', 'S', 'P'};
  CHECK(!sp::decode_info(p));
  p = {1};
  CHECK(!sp::decode_info(p));
  // an empty payload decodes to "nothing known"
  info = sp::decode_info({});
  CHECK(info && !info->chip_model && info->unknown_tags == 0);
  // a string longer than 255 bytes is truncated by the builder, not corrupted
  sp::InfoBuilder b;
  b.str(sp::InfoTag::ProjectName, std::string(300, 'p'));
  info = sp::decode_info(b.payload());
  CHECK(info && info->project_name && info->project_name->size() == 255);
}

static void test_requests_and_replies() {
  std::printf("test_requests_and_replies\n");
  const auto d = sp::encode_delay(750);
  CHECK(d.size() == 2 && d[0] == 0xEE && d[1] == 0x02);
  CHECK(sp::decode_delay(d) == 750);
  CHECK(sp::decode_delay({}) == 0); // empty payload = no delay
  const uint8_t one[] = {1};
  CHECK(!sp::decode_delay(one));
  const uint8_t three[] = {1, 2, 3}; // too long is malformed too, not "the first two bytes"
  CHECK(!sp::decode_delay(three));
  CHECK(sp::encode_ok(0x02) == std::vector<uint8_t>{0x02});
  const auto e = sp::encode_error(0x03, 95, "not supported");
  CHECK(e.size() == 5 + 13 && e[0] == 3 && e[1] == 95 && e[2] == 0 && e[5] == 'n');
  const auto de = sp::decode_error(e);
  CHECK(de && de->request_type == 3 && de->code == 95 && de->message == "not supported");
  CHECK(!sp::decode_error(std::span<const uint8_t>(e.data(), 4)));
  // frames: replies carry the reply flag, requests do not; module is stamped
  const auto req = sp::build_frame(sp::Type::Reboot, d, 7);
  CHECK(req[2] == 0x10 && req[3] == 7 && req[4] == 0x02);
  const auto rep = sp::build_frame(sp::Type::Info, {}, 11);
  CHECK(rep[2] == 0x11 && rep[3] == 11 && rep[4] == 0x81);
  sf::StreamParser parser;
  std::vector<uint8_t> stream(req);
  stream.insert(stream.end(), rep.begin(), rep.end());
  const auto frames = parser.feed(stream);
  CHECK(frames.size() == 2 && !frames[0].is_reply() && frames[0].payload.size() == 2 &&
        frames[1].is_reply() && frames[1].module == 11);
  // correlation: a request may carry a u16 id; every reply SystemService builds
  // (INFO, OK, ERROR) passes the request's id through build_frame, so it is
  // echoed; a request without one gets a reply without one
  const auto creq = sp::build_frame(sp::Type::GetInfo, {}, 7, 0x1234);
  const auto cf = sf::StreamParser{}.feed(creq);
  CHECK(cf.size() == 1 && cf[0].correlation == std::optional<uint16_t>(0x1234) &&
        (creq[2] & 0x02) != 0 && creq.size() == 11 + 4);
  for (const auto t : {sp::Type::Info, sp::Type::Ok, sp::Type::Error}) {
    const auto crep = sp::build_frame(t, sp::encode_ok(1), 7, cf[0].correlation);
    const auto cr = sf::StreamParser{}.feed(crep);
    CHECK(cr.size() == 1 && cr[0].is_reply() &&
          cr[0].correlation == std::optional<uint16_t>(0x1234));
  }
  const auto plain =
      sf::StreamParser{}.feed(sp::build_frame(sp::Type::Ok, sp::encode_ok(1), 7, std::nullopt));
  CHECK(plain.size() == 1 && !plain[0].has_correlation());
}

int main() {
  test_info_golden();
  test_info_all_tags();
  test_info_unknown_and_truncated();
  test_requests_and_replies();
  if (g_failures) {
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("ALL TESTS PASSED\n");
  return 0;
}
