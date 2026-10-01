// Host-buildable unit tests for the espp hid-rp report classes other than the
// gamepad (which hid_rp_gamepad_host_test.cpp covers): the bytes get_report()
// returns and set_data() consumes must be exactly the payload the descriptor
// declares, placed right after the report id when there is one and at byte 0
// when REPORT_ID == 0, for every report (3Dconnexion, gamepad LEDs, Xbox
// rumble / battery, Switch Pro, DualShock 4). hid-rp's own headers (and the
// intergatedcircuits/hid-rp headers it wraps) are not -Werror clean, so they
// are pulled in via -isystem. The Switch Pro header uses C++23 decay-copy
// (`auto(x)`) and the DualSense one static locals in constexpr functions, so
// build with g++ (>= 13) and -std=c++23:
//
//   g++ -std=c++23 -Wall -Wextra -Werror -Wno-comment
//       -isystem components/hid-rp/include
//       -isystem components/hid-rp/detail/hid-rp/hid-rp
//       -isystem components/format/include
//       -isystem components/format/detail/fmt/include
//       components/hid-rp/test/hid_rp_reports_host_test.cpp -o /tmp/hid_rp_reports_host_test
//       && /tmp/hid_rp_reports_host_test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "hid-rp-3dconnexion.hpp"
#include "hid-rp-gamepad.hpp"
#include "hid-rp-ps4.hpp"
#include "hid-rp-switch-pro.hpp"
#include "hid-rp-xbox.hpp"

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

// The raw object bytes: with a report id the first one must be that id and
// the payload must follow it immediately; without one the payload is the
// whole object.
template <typename Report> static std::vector<uint8_t> object_bytes(const Report &r) {
  const auto *p = reinterpret_cast<const uint8_t *>(
      &r); // (not data(): the DS4 output report has a `data` array)
  return std::vector<uint8_t>(p, p + sizeof(Report));
}

template <typename Report> static void check_offset_and_size(const Report &r, size_t payload) {
  const auto bytes = object_bytes(r);
  const auto report = r.get_report();
  CHECK(report.size() == payload);
  CHECK(Report::data_offset == (Report::ID != 0 ? 1u : 0u));
  CHECK(sizeof(Report) == Report::data_offset + payload);
  if (Report::ID != 0)
    CHECK(bytes[0] == Report::ID);
  for (size_t i = 0; i < payload && i < report.size(); ++i)
    CHECK(report[i] == bytes[Report::data_offset + i]);
}

static void test_spacemouse() {
  std::printf("test_spacemouse\n");
  espp::SpaceMouseTranslationInputReport<> t;
  // (values inside the +-350 logical range: the setters clamp)
  t.set_translation(0x0102, static_cast<std::int16_t>(-2), 0x0103);
  check_offset_and_size(t, 6);
  auto bytes = t.get_report();
  CHECK(bytes[0] == 0x02 && bytes[1] == 0x01); // X, little-endian
  CHECK(bytes[2] == 0xFE && bytes[3] == 0xFF); // Y = -2
  CHECK(bytes[4] == 0x03 && bytes[5] == 0x01); // Z
  // no report id: the same payload from byte 0
  using TranslationNoId = espp::SpaceMouseTranslationInputReport<-350, 350, 0>;
  TranslationNoId t0;
  t0.set_translation(0x0102, static_cast<std::int16_t>(-2), 0x0103);
  check_offset_and_size(t0, 6);
  CHECK(t0.get_report() == bytes);
  // set_data() round trip, and a short payload zero-fills the rest
  espp::SpaceMouseTranslationInputReport<> t2;
  t2.set_data(bytes);
  CHECK(t2.get_report() == bytes);
  t2.set_data({0x10, 0x00});
  CHECK(t2.get_report() == (std::vector<uint8_t>{0x10, 0x00, 0, 0, 0, 0}));

  espp::SpaceMouseRotationInputReport<> rot;
  rot.set_rotation(0x010B, static_cast<std::int16_t>(-300), 0x0105);
  check_offset_and_size(rot, 6);
  CHECK(rot.get_report()[0] == 0x0B && rot.get_report()[1] == 0x01);
  CHECK(rot.get_report()[2] == 0xD4 && rot.get_report()[3] == 0xFE); // -300
  CHECK(rot.get_report()[4] == 0x05 && rot.get_report()[5] == 0x01);
  using RotationNoId = espp::SpaceMouseRotationInputReport<-350, 350, 0>;
  RotationNoId rot0;
  rot0.set_rotation(0x010B, static_cast<std::int16_t>(-300), 0x0105);
  check_offset_and_size(rot0, 6);
  CHECK(rot0.get_report() == rot.get_report());

  espp::SpaceMouseButtonsInputReport<2> b;
  b.set_button(2, true);
  check_offset_and_size(b, 1);
  CHECK(b.get_report()[0] == 0x02);
  using ButtonsNoId = espp::SpaceMouseButtonsInputReport<2, 0>;
  ButtonsNoId b0;
  b0.set_button(2, true);
  check_offset_and_size(b0, 1);
  CHECK(b0.get_report()[0] == 0x02);

  espp::SpaceMouseLedOutputReport<> led;
  led.set_led(true);
  check_offset_and_size(led, 1);
  CHECK(led.get_report()[0] == 0x01);
  espp::SpaceMouseLedOutputReport<0> led0;
  led0.set_data({0x01});
  check_offset_and_size(led0, 1);
  CHECK(led0.get_led());
}

static void test_gamepad_leds() {
  std::printf("test_gamepad_leds\n");
  espp::GamepadLedOutputReport<> leds;
  leds.set_led(1, true);
  leds.set_led(4, true);
  auto bytes = leds.get_report();
  CHECK(bytes.size() == 1);
  CHECK(bytes[0] == 0x09);
  CHECK(object_bytes(leds)[0] == 2 && object_bytes(leds)[1] == 0x09);
  CHECK(espp::GamepadLedOutputReport<>::data_offset == 1);
  using LedsNoId = espp::GamepadLedOutputReport<4, 0>;
  LedsNoId leds0;
  leds0.set_data({0x09});
  CHECK(LedsNoId::data_offset == 0);
  CHECK(leds0.get_led(1) && !leds0.get_led(2) && !leds0.get_led(3) && leds0.get_led(4));
  CHECK(object_bytes(leds0)[0] == 0x09);
}

static void test_xbox() {
  std::printf("test_xbox\n");
  // the rumble payload is 8 bytes (the descriptor: two nibbles, four
  // magnitudes, duration, start delay, loop count); it used to be reported as
  // sizeof(report) = 9, reading one byte past the object
  espp::XboxRumbleOutputReport<> rumble;
  CHECK(espp::XboxRumbleOutputReport<>::num_data_bytes == 8);
  CHECK(sizeof(rumble) == 9);
  rumble.set_data({0x0F, 10, 20, 30, 40, 50, 60, 70});
  check_offset_and_size(rumble, 8);
  auto bytes = rumble.get_report();
  CHECK(bytes[0] == 0x0F && bytes[1] == 10 && bytes[4] == 40 && bytes[7] == 70);
  espp::XboxRumbleOutputReport<0> rumble0;
  CHECK(espp::XboxRumbleOutputReport<0>::num_data_bytes == 8);
  CHECK(sizeof(rumble0) == 8);
  rumble0.set_data({0x0F, 10, 20, 30, 40, 50, 60, 70});
  check_offset_and_size(rumble0, 8);
  CHECK(rumble0.get_report() == bytes);

  espp::XboxBatteryInputReport<> batt;
  batt.set_data({0xA5});
  check_offset_and_size(batt, 1);
  CHECK(batt.get_report()[0] == 0xA5);
  espp::XboxBatteryInputReport<0> batt0;
  batt0.set_data({0xA5});
  check_offset_and_size(batt0, 1);
  CHECK(batt0.get_report()[0] == 0xA5);
}

static void test_switch_pro() {
  std::printf("test_switch_pro\n");
  std::vector<uint8_t> payload(63);
  for (size_t i = 0; i < payload.size(); ++i)
    payload[i] = static_cast<uint8_t>(0x40 + i);
  espp::SwitchProGamepadInputReport<> sp;
  sp.set_data(payload);
  check_offset_and_size(sp, 63);
  CHECK(sp.get_report() == payload);
  espp::SwitchProGamepadInputReport<0> sp0;
  sp0.set_data(payload);
  check_offset_and_size(sp0, 63);
  CHECK(sp0.get_report() == payload);
}

static void test_ps4() {
  std::printf("test_ps4\n");
  // the payload is the 63 bytes the descriptor declares and starts with the
  // left stick X (the union begins at the first payload byte; the report id
  // is in the base class, not in raw)
  using Input = espp::PS4DualShock4GamepadInputReport<>;
  Input in;
  CHECK(Input::num_data_bytes == 63);
  in.set_left_joystick(0x11, 0x22);
  in.set_right_joystick(0x33, 0x44);
  in.set_l2_trigger(0x55);
  in.set_r2_trigger(0x66);
  auto bytes = in.get_report();
  CHECK(bytes.size() == 63);
  CHECK(bytes[0] == 0x11 && bytes[1] == 0x22 && bytes[2] == 0x33 && bytes[3] == 0x44);
  CHECK(bytes[7] == 0x55 && bytes[8] == 0x66); // after the 3 button bytes
  const auto obj = object_bytes(in);
  CHECK(obj[0] == 0x01 && obj[1] == 0x11); // id, then the payload
  // set_data() round trip from the wire (no id byte in the data)
  Input in2;
  in2.set_data(bytes);
  CHECK(in2.get_report() == bytes);
  // a longer vector than the payload is clamped, not written past raw
  std::vector<uint8_t> big(200, 0x7E);
  in2.set_data(big);
  CHECK(in2.get_report().size() == 63 && in2.get_report()[62] == 0x7E);

  using Output = espp::PS4DualShock4OutputReport<0x05>;
  Output out;
  CHECK(Output::num_data_bytes == 31);
  out.set_data({0x07, 0x00, 0x00, 0x80, 0x40});
  auto ob = out.get_report();
  CHECK(ob.size() == 31);
  CHECK(ob[0] == 0x07 && ob[3] == 0x80 && ob[4] == 0x40);
  CHECK(object_bytes(out)[0] == 0x05 && object_bytes(out)[1] == 0x07);
  // the BLE 0x11 report's rumble setters land at payload bytes 3 and 4
  espp::PS4DualShock4OutputReport<0x11> ble;
  ble.set_rumble(0xAA, 0xBB);
  CHECK(ble.get_report()[3] == 0xBB && ble.get_report()[4] == 0xAA);
}

int main() {
  test_spacemouse();
  test_gamepad_leds();
  test_xbox();
  test_switch_pro();
  test_ps4();
  if (g_failures) {
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("ALL TESTS PASSED\n");
  return 0;
}
