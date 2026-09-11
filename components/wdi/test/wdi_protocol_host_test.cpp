// Host-side unit test for the WDI (Wheelchair Digital Interface) protocol core.
// Builds with just a C++20 standard library (no ESP-IDF):
//
//   c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
//       components/wdi/test/wdi_protocol_host_test.cpp -o wdi_test && ./wdi_test

#include <cstdio>
#include <vector>

#include "detail/wdi_protocol.hpp"

namespace wdi = espp::wdi;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static void test_sizes_and_descriptor() {
  std::printf("test_sizes_and_descriptor\n");
  CHECK(wdi::kControlSize == 18);
  CHECK(wdi::kFeedbackSize == 19);
  CHECK(wdi::kKeepaliveResponseSize == 16);
  // Descriptor sanity: vendor usage page, application collection, ends with 0xC0,
  // and declares all five report IDs.
  const auto &d = wdi::kReportDescriptor;
  CHECK(d[0] == 0x06 && d[1] == 0x00 && d[2] == 0xFF); // Usage Page (Vendor 0xFF00)
  CHECK(d.back() == 0xC0);                             // End Collection
  int report_ids = 0;
  for (size_t i = 0; i + 1 < d.size(); ++i)
    if (d[i] == 0x85) // Report ID item
      ++report_ids;
  CHECK(report_ids == 5);
}

static void test_control_roundtrip() {
  std::printf("test_control_roundtrip\n");
  wdi::ControlReport c;
  c.x = -127;
  c.y = 100;
  c.set(wdi::ControlBit::DriveEnable);
  c.set(wdi::ControlBit::SpeedUp);
  c.set(wdi::ControlBit::Tilt);
  c.set(wdi::ControlBit::Modifier); // Tilt + Modifier = tilt backward
  c.vendor1 = 0xDEADBEEF;
  c.vendor2 = 0x01020304;

  const auto bytes = c.serialize();
  CHECK(bytes.size() == wdi::kControlSize);
  CHECK(static_cast<int8_t>(bytes[0]) == -127);
  CHECK(static_cast<int8_t>(bytes[1]) == 100);
  // standard1 little-endian at bytes 2..5.
  CHECK(bytes[2] == static_cast<uint8_t>(c.standard1));
  CHECK(bytes[5] == static_cast<uint8_t>(c.standard1 >> 24));
  // vendor1 little-endian at bytes 10..13 (0xDEADBEEF -> EF BE AD DE).
  CHECK(bytes[10] == 0xEF && bytes[11] == 0xBE && bytes[12] == 0xAD && bytes[13] == 0xDE);

  auto parsed = wdi::ControlReport::parse(bytes);
  CHECK(parsed.has_value());
  if (parsed.has_value()) {
    CHECK(parsed->x == -127 && parsed->y == 100);
    CHECK(parsed->has(wdi::ControlBit::DriveEnable));
    CHECK(parsed->has(wdi::ControlBit::SpeedUp));
    CHECK(parsed->has(wdi::ControlBit::Tilt));
    CHECK(parsed->has(wdi::ControlBit::Modifier));
    CHECK(!parsed->has(wdi::ControlBit::Stop));
    CHECK(parsed->vendor1 == 0xDEADBEEF && parsed->vendor2 == 0x01020304);
    CHECK(!parsed->is_release());
  }
}

static void test_control_release_and_bad_size() {
  std::printf("test_control_release_and_bad_size\n");
  wdi::ControlReport zero;
  CHECK(zero.is_release());
  const std::array<uint8_t, wdi::kControlSize> all_zero{};
  CHECK(zero.serialize() == all_zero);
  // Wrong-size payloads do not parse.
  std::vector<uint8_t> short_buf(wdi::kControlSize - 1, 0);
  CHECK(!wdi::ControlReport::parse(short_buf).has_value());
  std::vector<uint8_t> long_buf(wdi::kControlSize + 1, 0);
  CHECK(!wdi::ControlReport::parse(long_buf).has_value());
}

static void test_feedback_roundtrip() {
  std::printf("test_feedback_roundtrip\n");
  wdi::FeedbackReport f;
  f.set(wdi::FeedbackBit::DriveEnabled);
  f.set(wdi::FeedbackBit::ModeDrive);
  f.set(wdi::FeedbackBit::LimitedSpeed);
  f.speed = 5;
  f.profile = 2;
  f.velocity_whole = 3; // 3.7 mph
  f.velocity_tenths = 7;
  f.odometer = 42;

  const auto bytes = f.serialize();
  CHECK(bytes.size() == wdi::kFeedbackSize);
  // Byte 12: high nibble speed(5), low nibble profile(2) -> 0x52.
  CHECK(bytes[12] == 0x52);
  // Byte 13: high nibble whole(3), low nibble tenths(7) -> 0x37.
  CHECK(bytes[13] == 0x37);
  CHECK(bytes[14] == 42);
  CHECK(bytes[15] == 0 && bytes[18] == 0); // reserved stays zero

  auto parsed = wdi::FeedbackReport::parse(bytes);
  CHECK(parsed.has_value());
  if (parsed.has_value()) {
    CHECK(parsed->has(wdi::FeedbackBit::DriveEnabled));
    CHECK(parsed->has(wdi::FeedbackBit::ModeDrive));
    CHECK(parsed->has(wdi::FeedbackBit::LimitedSpeed));
    CHECK(!parsed->has(wdi::FeedbackBit::NoMovement));
    CHECK(parsed->speed == 5 && parsed->profile == 2);
    CHECK(parsed->velocity_whole == 3 && parsed->velocity_tenths == 7);
    // 3 + 7/10 = 3.7
    CHECK(parsed->velocity_mph() > 3.69f && parsed->velocity_mph() < 3.71f);
    CHECK(parsed->odometer == 42);
  }
}

static void test_feedback_nibble_clamping() {
  std::printf("test_feedback_nibble_clamping\n");
  // Values that would overflow a nibble are masked to 4 bits on serialize, so a
  // round-trip is stable within the valid range and never corrupts adjacent
  // nibbles.
  wdi::FeedbackReport f;
  f.speed = 15;
  f.profile = 15;
  f.velocity_whole = 15;
  f.velocity_tenths = 9;
  const auto bytes = f.serialize();
  CHECK(bytes[12] == 0xFF);
  CHECK(bytes[13] == 0xF9);
  auto parsed = wdi::FeedbackReport::parse(bytes);
  CHECK(parsed.has_value());
  if (parsed.has_value())
    CHECK(parsed->speed == 15 && parsed->profile == 15 && parsed->velocity_whole == 15 &&
          parsed->velocity_tenths == 9);
}

static void test_host_uuid() {
  std::printf("test_host_uuid\n");
  // Manufacturer id is big-endian in bytes 0..1: 0x000B = LUCI Mobility.
  std::array<uint8_t, wdi::kKeepaliveResponseSize> raw{};
  raw[0] = 0x00;
  raw[1] = 0x0B;
  raw[6] = 0x4A; // v4 marker in high nibble
  raw[8] = 0x9F; // top two bits 0b10
  auto u = wdi::HostUuid::parse(raw);
  CHECK(u.has_value());
  if (u.has_value()) {
    CHECK(u->manufacturer_id() == 0x000B);
    CHECK(u->manufacturer_id() == static_cast<uint16_t>(wdi::ManufacturerId::LuciMobility));
    CHECK(u->serialize() == raw); // stored verbatim (big-endian on the wire)
  }
  CHECK(!wdi::HostUuid::parse(std::vector<uint8_t>(15, 0)).has_value());
}

int main() {
  test_sizes_and_descriptor();
  test_control_roundtrip();
  test_control_release_and_bad_size();
  test_feedback_roundtrip();
  test_feedback_nibble_clamping();
  test_host_uuid();
  if (g_failures == 0) {
    std::printf("ALL WDI PROTOCOL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d FAILURE(S)\n", g_failures);
  return 1;
}
