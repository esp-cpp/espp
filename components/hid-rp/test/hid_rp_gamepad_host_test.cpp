// Host-buildable unit tests for the espp hid-rp gamepad input report
// (hid-rp-gamepad.hpp): the bytes get_report() returns and set_data() consumes
// must line up with the fields the report descriptor declares, for byte-wide
// axes and for no report id, not just the default uint16_t / id 1 layout.
// hid-rp's own headers (and the intergatedcircuits/hid-rp headers it wraps)
// are not -Werror clean, so they are pulled in via -isystem. Build & run:
//
//   c++ -std=c++20 -Wall -Wextra -Werror \
//       -isystem components/hid-rp/include \
//       -isystem components/hid-rp/detail/hid-rp/hid-rp \
//       -isystem components/format/include \
//       -isystem components/format/detail/fmt/include \
//       components/hid-rp/test/hid_rp_gamepad_host_test.cpp -o
//       /tmp/hid_rp_gamepad_host_test \
//       && /tmp/hid_rp_gamepad_host_test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "hid-rp-gamepad.hpp"

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// Little-endian unsigned 16-bit helper, matching the wire format of the
// default report's axes and triggers.
static uint16_t le16(const std::vector<uint8_t> &data, size_t index) {
  return static_cast<uint16_t>(data[index] | (data[index + 1] << 8));
}

// The default report: 15 buttons, uint16_t axes, 10-bit triggers, report id 1.
using DefaultReport = espp::GamepadInputReport<>;
// Byte-wide axes and triggers, as an 8-bit joystick (e.g. one built for the
// Xbox Adaptive Controller) uses. There is no padding after the report id
// here, so the data starts one byte earlier than in the default report.
using ByteReport = espp::GamepadInputReport<12, uint8_t, uint8_t, 0, 255, 0, 255, 1>;
// The same report with no report id at all.
using ByteReportNoId = espp::GamepadInputReport<12, uint8_t, uint8_t, 0, 255, 0, 255, 0>;

static void test_default_layout() {
  std::printf("test_default_layout\n");
  DefaultReport report;
  report.set_joystick_axis(0, uint16_t{0x0102}); // X
  report.set_joystick_axis(1, uint16_t{0x0304}); // Y
  report.set_joystick_axis(2, uint16_t{0x0506}); // Z
  report.set_joystick_axis(3, uint16_t{0x0708}); // RZ
  report.set_trigger_axis(0, uint16_t{0x0123});  // brake
  report.set_trigger_axis(1, uint16_t{0x0345});  // accelerator
  report.set_hat(DefaultReport::Hat::UP_RIGHT);
  report.set_button(1, true);
  report.set_button(15, true);
  report.set_consumer_record(true);

  auto bytes = report.get_report();
  // 4 x uint16 axes, 2 x uint16 triggers, hat, 2 button bytes, consumer byte
  CHECK(bytes.size() == 16);
  CHECK(le16(bytes, 0) == 0x0102);
  CHECK(le16(bytes, 2) == 0x0304);
  CHECK(le16(bytes, 4) == 0x0506);
  CHECK(le16(bytes, 6) == 0x0708);
  CHECK(le16(bytes, 8) == 0x0123);
  CHECK(le16(bytes, 10) == 0x0345);
  CHECK((bytes[12] & 0x0f) == static_cast<uint8_t>(DefaultReport::Hat::UP_RIGHT));
  CHECK(bytes[13] == 0x01);
  CHECK(bytes[14] == 0x40);
  CHECK((bytes[15] & 0x01) == 0x01);
}

template <typename Report> static void check_byte_layout() {
  Report report;
  report.set_joystick_axis(0, uint8_t{0xA1}); // X
  report.set_joystick_axis(1, uint8_t{0xA2}); // Y
  report.set_joystick_axis(2, uint8_t{0xA3}); // Z
  report.set_joystick_axis(3, uint8_t{0xA4}); // RZ
  report.set_trigger_axis(0, uint8_t{0xB1});  // brake
  report.set_trigger_axis(1, uint8_t{0xB2});  // accelerator
  report.set_hat(Report::Hat::UP_RIGHT);
  report.set_button(1, true);
  report.set_button(12, true);
  report.set_consumer_record(true);

  auto bytes = report.get_report();
  // 4 x uint8 axes, 2 x uint8 triggers, hat, 2 button bytes, consumer byte
  CHECK(bytes.size() == 10);
  CHECK(bytes[0] == 0xA1);
  CHECK(bytes[1] == 0xA2);
  CHECK(bytes[2] == 0xA3);
  CHECK(bytes[3] == 0xA4);
  CHECK(bytes[4] == 0xB1);
  CHECK(bytes[5] == 0xB2);
  CHECK((bytes[6] & 0x0f) == static_cast<uint8_t>(Report::Hat::UP_RIGHT));
  CHECK(bytes[7] == 0x01);
  CHECK(bytes[8] == 0x08);
  CHECK((bytes[9] & 0x01) == 0x01);
}

static void test_byte_axes_layout() {
  std::printf("test_byte_axes_layout\n");
  check_byte_layout<ByteReport>();
}

static void test_byte_axes_layout_without_report_id() {
  std::printf("test_byte_axes_layout_without_report_id\n");
  check_byte_layout<ByteReportNoId>();
}

static void test_byte_axes_centered_is_neutral() {
  std::printf("test_byte_axes_centered_is_neutral\n");
  ByteReport report;
  report.set_left_joystick(0.0f, 0.0f);
  report.set_right_joystick(0.0f, 0.0f);

  auto bytes = report.get_report();
  CHECK(bytes.size() == 10);
  for (size_t i = 0; i < 4; i++) {
    CHECK(bytes[i] == ByteReport::joystick_center);
  }
  CHECK(bytes[4] == 0);
  CHECK(bytes[5] == 0);
  // 0x0f is outside the hat's 1..8 logical range: the null state, no d-pad
  CHECK((bytes[6] & 0x0f) == static_cast<uint8_t>(ByteReport::Hat::CENTERED));
  CHECK(bytes[7] == 0);
  CHECK(bytes[8] == 0);
  CHECK(bytes[9] == 0);
}

template <typename Report, typename AxisType> static void check_round_trip(AxisType x, AxisType y) {
  Report report;
  report.set_joystick_axis(0, x);
  report.set_joystick_axis(1, y);
  report.set_hat(Report::Hat::DOWN_LEFT);
  report.set_button(3, true);
  auto bytes = report.get_report();

  Report decoded;
  decoded.set_data(bytes);
  AxisType dx, dy;
  decoded.get_left_joystick(dx, dy);
  CHECK(dx == x);
  CHECK(dy == y);
  CHECK(decoded.get_button(3));
  CHECK(!decoded.get_button(4));
  CHECK(decoded.get_report() == bytes);
}

static void test_set_data_round_trip() {
  std::printf("test_set_data_round_trip\n");
  check_round_trip<DefaultReport>(uint16_t{0x1234}, uint16_t{0xABCD});
  check_round_trip<ByteReport>(uint8_t{0x12}, uint8_t{0xAB});
  check_round_trip<ByteReportNoId>(uint8_t{0x12}, uint8_t{0xAB});
}

int main() {
  test_default_layout();
  test_byte_axes_layout();
  test_byte_axes_layout_without_report_id();
  test_byte_axes_centered_is_neutral();
  test_set_data_round_trip();

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  } else {
    std::printf("%d CHECK(S) FAILED\n", g_failures);
    return 1;
  }
}
