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
  test_malformed_reply_payloads();
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
