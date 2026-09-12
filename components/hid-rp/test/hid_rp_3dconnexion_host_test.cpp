// Host-buildable unit tests for the espp hid-rp 3Dconnexion SpaceMouse report
// descriptor (hid-rp-3dconnexion.hpp). hid-rp's own headers (and the
// intergatedcircuits/hid-rp headers it wraps) are not -Werror clean, so they
// are pulled in via -isystem. Build & run:
//
//   c++ -std=c++20 -Wall -Wextra -Werror \
//       -isystem components/hid-rp/include \
//       -isystem components/hid-rp/detail/hid-rp/hid-rp \
//       -isystem components/format/include \
//       -isystem components/format/detail/fmt/include \
//       components/hid-rp/test/hid_rp_3dconnexion_host_test.cpp -o
//       /tmp/hid_rp_3dconnexion_host_test \
//       && /tmp/hid_rp_3dconnexion_host_test
//
// No ESP-IDF headers required.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <vector>

#include "hid-rp-3dconnexion.hpp"

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// Little-endian signed 16-bit helper, matching the wire format of the
// translation/rotation reports.
static int16_t le16(const std::vector<uint8_t> &data, size_t index) {
  uint16_t raw = static_cast<uint16_t>(data[index]) | (static_cast<uint16_t>(data[index + 1]) << 8);
  return static_cast<int16_t>(raw);
}

static void test_translation_report() {
  std::printf("test_translation_report\n");
  espp::SpaceMouseTranslationInputReport<> report;

  // default state is centered (0)
  CHECK(report.get_x() == 0);
  CHECK(report.get_y() == 0);
  CHECK(report.get_z() == 0);

  report.set_translation(350, -350, 123);
  CHECK(report.get_x() == 350);
  CHECK(report.get_y() == -350);
  CHECK(report.get_z() == 123);

  // clamping to the logical range
  report.set_x(1000);
  CHECK(report.get_x() == 350);
  report.set_y(-1000);
  CHECK(report.get_y() == -350);

  auto bytes = report.get_report();
  CHECK(bytes.size() == 6); // 3 x int16_t, no report id byte
  int16_t rx, ry, rz;
  report.get_translation(rx, ry, rz);
  CHECK(le16(bytes, 0) == rx);
  CHECK(le16(bytes, 2) == ry);
  CHECK(le16(bytes, 4) == rz);

  // round trip through set_data()
  espp::SpaceMouseTranslationInputReport<> report2;
  report2.set_data(bytes);
  CHECK(report2.get_x() == report.get_x());
  CHECK(report2.get_y() == report.get_y());
  CHECK(report2.get_z() == report.get_z());

  report.reset();
  CHECK(report.get_x() == 0 && report.get_y() == 0 && report.get_z() == 0);
}

static void test_rotation_report() {
  std::printf("test_rotation_report\n");
  espp::SpaceMouseRotationInputReport<> report;

  report.set_rotation(-100, 200, -300);
  CHECK(report.get_rx() == -100);
  CHECK(report.get_ry() == 200);
  CHECK(report.get_rz() == -300);

  auto bytes = report.get_report();
  CHECK(bytes.size() == 6);
  CHECK(le16(bytes, 0) == -100);
  CHECK(le16(bytes, 2) == 200);
  CHECK(le16(bytes, 4) == -300);
}

static void test_buttons_report() {
  std::printf("test_buttons_report\n");
  // default (SpaceNavigator): 2 buttons
  espp::SpaceMouseButtonsInputReport<> report;
  CHECK(report.get_report().size() == 1); // 2 buttons -> 1 byte, byte-padded

  CHECK(!report.get_button(1));
  CHECK(!report.get_button(2));
  report.set_button(1, true);
  CHECK(report.get_button(1));
  CHECK(!report.get_button(2));
  auto bytes = report.get_report();
  CHECK((bytes[0] & 0x01) != 0);
  CHECK((bytes[0] & 0x02) == 0);

  report.set_button(2, true);
  bytes = report.get_report();
  CHECK(bytes[0] == 0x03);

  // out of range indices are ignored/false
  report.set_button(0, true);
  report.set_button(99, true);
  CHECK(!report.get_button(0));
  CHECK(!report.get_button(99));

  report.reset();
  CHECK(report.get_report()[0] == 0x00);

  // a higher button-count model (e.g. SpaceMouse Pro Enterprise-ish)
  espp::SpaceMouseButtonsInputReport<15> pro_buttons;
  CHECK(pro_buttons.get_report().size() == 2); // 15 buttons -> 2 bytes, byte-padded
  pro_buttons.set_button(15, true);
  CHECK(pro_buttons.get_button(15));
  CHECK(pro_buttons.get_report()[1] == 0x40);
}

static void test_led_report() {
  std::printf("test_led_report\n");
  espp::SpaceMouseLedOutputReport<> report;
  CHECK(!report.get_led());
  report.set_led(true);
  CHECK(report.get_led());
  auto bytes = report.get_report();
  CHECK(bytes.size() == 1);
  CHECK(bytes[0] == 0x01);
  report.reset();
  CHECK(!report.get_led());
}

static void test_descriptor() {
  std::printf("test_descriptor\n");

  auto raw_descriptor = espp::spacemouse_descriptor<>();
  std::vector<uint8_t> descriptor(raw_descriptor.begin(), raw_descriptor.end());
  std::printf("  SpaceMouse report descriptor size: %zu bytes\n", descriptor.size());
  // The fixed-index accesses below reach up to descriptor[3], so require at
  // least 4 bytes before indexing into it.
  CHECK(descriptor.size() >= 4);

  // Usage Page (Generic Desktop), Usage (Multi-Axis Controller)
  CHECK(descriptor[0] == 0x05 && descriptor[1] == 0x01);
  CHECK(descriptor[2] == 0x09 && descriptor[3] == 0x08);

  // Report ID 1 (translation): verify the verified-against-hardware logical
  // and physical limit bytes appear in the stream (16-bit signed -350/350
  // and -1400/1400, little endian).
  auto contains = [&](std::initializer_list<uint8_t> pattern) {
    return std::search(descriptor.begin(), descriptor.end(), pattern.begin(), pattern.end()) !=
           descriptor.end();
  };
  CHECK(contains({0x85, 0x01}));       // Report ID (1)
  CHECK(contains({0x16, 0xa2, 0xfe})); // Logical Minimum (-350)
  CHECK(contains({0x26, 0x5e, 0x01})); // Logical Maximum (350)
  CHECK(contains({0x36, 0x88, 0xfa})); // Physical Minimum (-1400)
  CHECK(contains({0x46, 0x78, 0x05})); // Physical Maximum (1400)
  CHECK(contains({0x85, 0x02}));       // Report ID (2, rotation)
  CHECK(contains({0x85, 0x03}));       // Report ID (3, buttons)
  CHECK(contains({0x85, 0x04}));       // Report ID (4, LED)

  // A button count that lands on a byte boundary (e.g. 8) needs no padding
  // item, unlike the default 2-button SpaceNavigator layout, so the two
  // descriptors differ in length.
  auto raw_descriptor_8 = espp::spacemouse_descriptor<8>();
  std::vector<uint8_t> descriptor_8(raw_descriptor_8.begin(), raw_descriptor_8.end());
  CHECK(descriptor_8.size() != descriptor.size());
}

int main() {
  test_translation_report();
  test_rotation_report();
  test_buttons_report();
  test_led_report();
  test_descriptor();

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  } else {
    std::printf("%d CHECK(S) FAILED\n", g_failures);
    return 1;
  }
}
