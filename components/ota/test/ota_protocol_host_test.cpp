// Host-buildable unit tests for the espp OTA stream protocol helpers. The raw
// frame codec (magic/flags/module/type/len/crc, StreamParser resync, ...) is
// tested by components/stream_frame/test; this file exercises the OTA-specific
// make_*/parse_* helpers layered on top. Build & run:
//   c++ -std=c++20 -Werror -I components/ota/include -I components/stream_frame/include \
//       components/ota/test/ota_protocol_host_test.cpp -o test && ./test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "detail/ota_stream_protocol.hpp"

namespace ota = espp::detail::ota_stream;
using ota::MessageType;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// Parse a single OTA frame out of an encoded buffer.
static bool parse_one(const std::vector<uint8_t> &encoded, ota::Frame &out) {
  ota::StreamParser parser;
  auto frames = parser.feed(encoded);
  if (frames.size() != 1)
    return false;
  out = frames[0];
  return true;
}

static void test_requests_are_module0_requests() {
  std::printf("test_requests_are_module0_requests\n");
  struct Case {
    std::vector<uint8_t> frame;
    MessageType type;
  };
  const uint8_t img[] = {0xE9, 0x06, 0x02};
  const Case cases[] = {
      {ota::make_begin(1234567u), MessageType::Begin},
      {ota::make_data(img), MessageType::Data},
      {ota::make_finish(), MessageType::Finish},
      {ota::make_abort(), MessageType::Abort},
      {ota::make_get_status(), MessageType::GetStatus},
      {ota::make_mark_valid(), MessageType::MarkValid},
      {ota::make_mark_invalid(), MessageType::MarkInvalid},
  };
  for (const auto &c : cases) {
    ota::Frame f{};
    CHECK(parse_one(c.frame, f));
    CHECK(f.module == ota::kModule);
    CHECK(f.type == static_cast<uint8_t>(c.type));
    CHECK(!f.is_reply()); // requests are host -> device
  }
  // BEGIN payload round-trips as a u32 image size.
  ota::Frame begin{};
  CHECK(parse_one(ota::make_begin(1234567u), begin));
  CHECK(ota::parse_u32_payload(begin).value_or(0) == 1234567u);
  // DATA payload is the raw image bytes.
  ota::Frame data{};
  CHECK(parse_one(ota::make_data(img), data));
  CHECK(data.payload.size() == sizeof(img) &&
        std::memcmp(data.payload.data(), img, sizeof(img)) == 0);
}

static void test_replies_carry_reply_flag() {
  std::printf("test_replies_carry_reply_flag\n");
  ota::Frame ok{}, err{}, prog{};
  CHECK(parse_one(ota::make_ok(6u), ok));
  CHECK(ok.module == ota::kModule && ok.type == static_cast<uint8_t>(MessageType::Ok));
  CHECK(ok.is_reply()); // replies are device -> host
  CHECK(ota::parse_u32_payload(ok).value_or(0) == 6u);

  CHECK(parse_one(ota::make_error(5u, "flash write failed"), err));
  CHECK(err.is_reply() && err.type == static_cast<uint8_t>(MessageType::Error));
  const auto info = ota::parse_error(err);
  CHECK(info.has_value());
  if (info.has_value()) {
    CHECK(info->code == 5u);
    CHECK(info->message == "flash write failed");
  }

  CHECK(parse_one(ota::make_progress(4096u, 8192u), prog));
  CHECK(prog.is_reply() && prog.type == static_cast<uint8_t>(MessageType::Progress));
  const auto p = ota::parse_progress(prog);
  CHECK(p.has_value());
  if (p.has_value()) {
    CHECK(p->written == 4096u && p->total == 8192u);
  }
}

static void test_status_reply() {
  std::printf("test_status_reply\n");
  // Full STATUS: both flags set + version + project round-trip.
  ota::Frame s{};
  CHECK(parse_one(ota::make_status(ota::kStatusPendingVerify | ota::kStatusRollbackSupported,
                                   "v1.2.3", "ota_example"),
                  s));
  CHECK(s.is_reply() && s.type == static_cast<uint8_t>(MessageType::Status));
  auto info = ota::parse_status(s);
  CHECK(info.has_value());
  if (info.has_value()) {
    CHECK(info->pending_verify());
    CHECK(info->rollback_supported());
    CHECK(info->version == "v1.2.3");
    CHECK(info->project_name == "ota_example");
  }

  // Flags-only STATUS (older device: no strings) still parses; strings empty.
  ota::Frame s2{};
  CHECK(parse_one(ota::make_status(0), s2));
  auto info2 = ota::parse_status(s2);
  CHECK(info2.has_value());
  if (info2.has_value()) {
    CHECK(!info2->pending_verify());
    CHECK(!info2->rollback_supported());
    CHECK(info2->version.empty() && info2->project_name.empty());
  }

  // Only the rollback-supported flag set (a confirmed image on a rollback build).
  ota::Frame s3{};
  CHECK(parse_one(ota::make_status(ota::kStatusRollbackSupported, "v2", "p"), s3));
  auto info3 = ota::parse_status(s3);
  CHECK(info3.has_value());
  if (info3.has_value()) {
    CHECK(!info3->pending_verify() && info3->rollback_supported());
  }
}

static void test_malformed_status_payloads() {
  std::printf("test_malformed_status_payloads\n");
  // Empty payload -> nullopt (no flags byte).
  ota::Frame empty{};
  CHECK(!ota::parse_status(empty).has_value());

  // Flags byte only -> valid, empty strings.
  ota::Frame flags_only{};
  flags_only.payload = {ota::kStatusPendingVerify};
  auto a = ota::parse_status(flags_only);
  CHECK(a.has_value());
  if (a.has_value()) {
    CHECK(a->pending_verify() && a->version.empty() && a->project_name.empty());
  }

  // Truncated version string (declares len 5, only 2 bytes present): clamp, don't
  // over-read, and leave the project empty.
  ota::Frame trunc{};
  trunc.payload = {0x02 /*flags*/, 0x05 /*version len*/, 'v', '2'};
  auto b = ota::parse_status(trunc);
  CHECK(b.has_value());
  if (b.has_value()) {
    CHECK(b->rollback_supported());
    CHECK(b->version == "v2"); // clamped to the available bytes
    CHECK(b->project_name.empty());
  }

  // Version present, project length declared but zero bytes follow.
  ota::Frame missing_proj{};
  missing_proj.payload = {0x00, 0x01, 'x', 0x04 /*project len, no bytes*/};
  auto c = ota::parse_status(missing_proj);
  CHECK(c.has_value());
  if (c.has_value()) {
    CHECK(c->version == "x" && c->project_name.empty());
  }
}

static void test_malformed_reply_payloads() {
  std::printf("test_malformed_reply_payloads\n");
  ota::Frame f{};
  f.payload = {0x01, 0x02};
  CHECK(!ota::parse_u32_payload(f).has_value()); // needs exactly 4 bytes
  ota::Frame e{};
  e.payload = {0x01, 0x02, 0x03};
  CHECK(!ota::parse_error(e).has_value()); // needs >= 4 bytes
  ota::Frame p{};
  p.payload = {0x01, 0x02, 0x03, 0x04};
  CHECK(!ota::parse_progress(p).has_value()); // needs exactly 8 bytes
  // An ERROR with just a code (no message) is valid.
  ota::Frame e2{};
  e2.payload = {0x05, 0x00, 0x00, 0x00};
  const auto info = ota::parse_error(e2);
  CHECK(info.has_value());
  if (info.has_value()) {
    CHECK(info->code == 5u && info->message.empty());
  }
}

int main() {
  test_requests_are_module0_requests();
  test_replies_carry_reply_flag();
  test_status_reply();
  test_malformed_status_payloads();
  test_malformed_reply_payloads();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
