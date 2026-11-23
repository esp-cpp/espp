// Host-buildable unit tests for the rumble helpers (hid-rp-rumble.hpp) and the
// Output-field side of the runtime report map. Build & run:
//
//   c++ -std=c++23 -Wall -Wextra -Werror -Wno-comment \
//       -isystem components/hid-rp/include \
//       -isystem components/hid-rp/detail/hid-rp/hid-rp \
//       -isystem components/format/include \
//       -isystem components/format/detail/fmt/include \
//       components/hid-rp/test/hid_rp_rumble_host_test.cpp \
//       -o /tmp/hid_rp_rumble_host_test \
//   && /tmp/hid_rp_rumble_host_test
//
// No ESP-IDF headers required.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "hid-rp-report-map.hpp"
#include "hid-rp-rumble.hpp"
#include "hid-rp-switch-pro.hpp"
#include "hid-rp-xbox.hpp"

static int failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      failures++;                                                                                  \
    }                                                                                              \
  } while (0)

using namespace espp::hid_rp;
namespace u = espp::hid_rp::usage;

template <typename D> static std::vector<uint8_t> bytes_of(const D &d) {
  return std::vector<uint8_t>(d.begin(), d.end());
}

static const ReportField *find_output(const ReportMap &m, uint8_t id, uint16_t usage, int nth = 0) {
  const auto &outs = m.output_fields();
  const auto it = std::find_if(outs.begin(), outs.end(), [&](const ReportField &f) {
    return f.report_id == id && f.usage_page == u::PAGE_PHYSICAL_INPUT_DEVICE && f.usage == usage &&
           nth-- == 0;
  });
  return it == outs.end() ? nullptr : &*it;
}

// --- the Xbox Wireless controller's report map ------------------------------
static void test_xbox_output_fields() {
  const auto desc = bytes_of(espp::xbox_descriptor());
  const auto map = ReportMap::parse(desc);
  CHECK(map.has_value());
  if (!map)
    return;
  // the rumble output report (id 3): 8 bytes, PID usages at the xpadneo offsets
  CHECK(map->output_report_bytes(3) == 8);
  const auto *enable = find_output(*map, 3, u::PID_DC_ENABLE_ACTUATORS);
  CHECK(enable && enable->bit_offset == 0 && enable->bit_size == 4);
  for (int i = 0; i < 4; i++) {
    const auto *m = find_output(*map, 3, u::PID_MAGNITUDE, i);
    CHECK(m && m->bit_offset == 8u + 8u * i && m->bit_size == 8 && m->logical_max == 100);
  }
  const auto *duration = find_output(*map, 3, u::PID_DURATION);
  CHECK(duration && duration->bit_offset == 40 && duration->unit == 0x1001 &&
        duration->unit_exponent == -2);
  const auto *delay = find_output(*map, 3, u::PID_START_DELAY);
  CHECK(delay && delay->bit_offset == 48);
  const auto *loop = find_output(*map, 3, u::PID_LOOP_COUNT);
  CHECK(loop && loop->bit_offset == 56 && loop->unit == 0);
  // outputs leave the input side alone: no input field from report 3
  for (const auto &f : map->fields())
    CHECK(f.report_id != 3);
  CHECK(map->uses_report_ids());
}

static void test_xbox_encoder() {
  const auto desc = bytes_of(espp::xbox_descriptor());
  const auto map = ReportMap::parse(desc);
  const auto enc = map ? RumbleEncoder::from_report_map(*map) : std::nullopt;
  CHECK(enc.has_value());
  if (!enc)
    return;
  CHECK(enc->report_id() == 3);
  CHECK(enc->report_size() == 8);
  CHECK(enc->actuator_count() == 4);
  using M = RumbleEncoder::Motor;
  CHECK(enc->motor(0) == M::LeftTrigger && enc->motor(1) == M::RightTrigger &&
        enc->motor(2) == M::Strong && enc->motor(3) == M::Weak);
  CHECK(enc->max_duration_ms() == 2550); // 255 x 10 ms

  // strong full, weak half, for 250 ms
  const auto b = enc->encode({.strong = 255, .weak = 128}, 250);
  const std::vector<uint8_t> want = {0x03, 0, 0, 100, 50, 25, 0, 0};
  CHECK(b == want);
  // byte for byte what espp's hand-written Xbox rumble report produces
  espp::XboxRumbleOutputReport<> r;
  r.set_enabled(0x03); // weak (bit 0) + strong (bit 1)
  r.set_magnitude(2, uint8_t{100});
  r.set_magnitude(3, uint8_t{50});
  r.set_duration(uint8_t{25});
  r.set_start_delay(uint8_t{0});
  r.set_loop_count(uint8_t{0});
  const auto ref = r.get_report();
  CHECK(std::vector<uint8_t>(ref.begin(), ref.end()) == b);

  // trigger motors: slot 0 (left trigger) is enable bit 3, slot 1 bit 2
  const auto t = enc->encode({.left_trigger = 255, .right_trigger = 255}, 100);
  CHECK(t.size() == 8 && t[0] == 0x0C && t[1] == 100 && t[2] == 100 && t[3] == 0 && t[4] == 0 &&
        t[5] == 10);
  // durations beyond the field clamp to it
  CHECK(enc->encode({.weak = 255}, 10'000)[5] == 255);
  // the stop report is all zero
  CHECK(enc->encode_stop() == std::vector<uint8_t>(8, 0));
  CHECK(enc->encode({}, 500) == std::vector<uint8_t>(8, 0));

  // a remapped actuator follows its new motor
  auto e2 = *enc;
  e2.set_motor(2, M::Weak);
  e2.set_motor(3, M::Strong);
  const auto s = e2.encode({.strong = 255, .weak = 0}, 0);
  CHECK(s[3] == 0 && s[4] == 100 && s[0] == 0x01); // weak slot (3) is bit 0
}

// --- a synthetic 2-motor PID layout -----------------------------------------
// Input: report 1, 8 buttons. Output: report 5, a 1-bit DC Enable flag + 7 bits
// of padding, two 16-bit magnitudes 0..10000, a 16-bit Duration with no unit
// (milliseconds).
static const uint8_t TWO_MOTOR_DESC[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01,                         // GD / Gamepad / Application
    0x85, 0x01,                                                 // Report ID 1
    0x05, 0x09, 0x19, 0x01, 0x29, 0x08, 0x15, 0x00, 0x25, 0x01, // buttons 1..8
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,                         //   Input (Data, Var)
    0x85, 0x05,                                                 // Report ID 5
    0x05, 0x0F,                                                 // PID page
    0x09, 0x97, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01, // DC Enable Actuators, 1 bit
    0x91, 0x02,                                                 //   Output (Data, Var)
    0x75, 0x07, 0x95, 0x01, 0x91, 0x03,                         // 7 bits padding (Cnst)
    0x09, 0x70, 0x15, 0x00, 0x26, 0x10, 0x27,                   // Magnitude 0..10000
    0x75, 0x10, 0x95, 0x02, 0x91, 0x02,                         //   x2, 16 bit
    0x09, 0x50, 0x15, 0x00, 0x26, 0xFF, 0x7F,                   // Duration 0..32767 (no unit)
    0x75, 0x10, 0x95, 0x01, 0x91, 0x02,                         //   16 bit
    0xC0};

static void test_two_motor_encoder() {
  const auto map = ReportMap::parse(TWO_MOTOR_DESC);
  CHECK(map.has_value());
  if (!map)
    return;
  CHECK(map->report_bytes(1) == 1);        // the input side is unchanged by the outputs
  CHECK(map->output_report_bytes(5) == 7); // 1 + 7 + 32 + 16 bits
  const auto enc = RumbleEncoder::from_report_map(*map);
  CHECK(enc.has_value());
  if (!enc)
    return;
  CHECK(enc->report_id() == 5 && enc->report_size() == 7 && enc->actuator_count() == 2);
  CHECK(enc->motor(0) == RumbleEncoder::Motor::Strong &&
        enc->motor(1) == RumbleEncoder::Motor::Weak);
  CHECK(enc->max_duration_ms() == 32767);
  const auto b = enc->encode({.strong = 255, .weak = 51}, 300);
  // enable flag (1 bit, narrower than the 2 actuators: a single on/off flag),
  // 10000 = 0x2710, 2000 = 0x07D0, 300 ms = 0x012C
  const std::vector<uint8_t> want = {0x01, 0x10, 0x27, 0xD0, 0x07, 0x2C, 0x01};
  CHECK(b == want);
  CHECK(enc->encode_stop() == std::vector<uint8_t>(7, 0));
}

// --- a device without a PID rumble report -----------------------------------
static void test_no_rumble_report() {
  // the Switch Pro descriptor has output reports (0x01, 0x10, 0x80, 0x82), but
  // each is 63 constant vendor bytes: walked (their size is known) yet no data
  // field, and nothing on the PID page
  const auto desc = bytes_of(espp::switch_pro_descriptor());
  const auto map = ReportMap::parse(desc);
  CHECK(map.has_value());
  if (map) {
    CHECK(map->output_report_bytes(0x01) == 63 && map->output_report_bytes(0x10) == 63);
    CHECK(map->output_fields().empty());
    CHECK(!RumbleEncoder::from_report_map(*map).has_value());
  }
}

// --- Switch host rumble decoding --------------------------------------------
static void test_switch_rumble() {
  // neutral packet: 00 01 40 40 per motor
  const uint8_t neutral[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
  CHECK(decode_switch_rumble(neutral) == SwitchRumble{});
  // left: HF amplitude at its maximum code (100); right: LF amplitude at its
  // maximum (0x72 with the half-step bit in byte 2)
  const uint8_t full[8] = {0x00, 0xC9, 0x40, 0x40, 0x00, 0x01, 0xC0, 0x72};
  const auto f = decode_switch_rumble(full);
  CHECK(f.left == 255 && f.right == 255);
  // HF code 64: 2^(64/32) / 8.7 = 0.46 -> 117
  const uint8_t mid[8] = {0x00, 0x80, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
  CHECK(decode_switch_rumble(mid) == (SwitchRumble{117, 0}));
  // the level grows with the code across the segment boundaries (16, 32)
  int last = -1;
  for (int code = 0; code <= 100; code++) {
    const uint8_t p[8] = {0x00, static_cast<uint8_t>(code << 1), 0x40, 0x40, 0x00, 0x01, 0x40,
                          0x40};
    const int level = decode_switch_rumble(p).left;
    CHECK(level >= last);
    last = level;
  }
  CHECK(last == 255);

  // whole reports: the rumble data follows the report id AND the packet number
  // (a 0xC9 packet number must not read as a full HF amplitude)
  const uint8_t r10[10] = {0x10, 0xC9, 0x00, 0x01, 0x40, 0x40, 0x00, 0x80, 0x40, 0x40};
  const auto a = decode_switch_rumble_report(r10);
  CHECK(a.has_value() && a->left == 0 && a->right == 117);
  const uint8_t r01[12] = {0x01, 0x05, 0x00, 0x80, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40, 0x48, 0x01};
  const auto b = decode_switch_rumble_report(r01);
  CHECK(b.has_value() && b->left == 117 && b->right == 0);
  const uint8_t init[10] = {0x80, 0x02, 0, 0, 0, 0, 0, 0, 0, 0}; // a USB init command
  CHECK(!decode_switch_rumble_report(init).has_value());
  const uint8_t short_report[9] = {0x10, 0x00, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40};
  CHECK(!decode_switch_rumble_report(short_report).has_value());
}

int main() {
  test_xbox_output_fields();
  test_xbox_encoder();
  test_two_motor_encoder();
  test_no_rumble_report();
  test_switch_rumble();
  if (failures == 0)
    std::printf("ALL HID-RP RUMBLE TESTS PASSED\n");
  else
    std::printf("%d FAILURE(S)\n", failures);
  return failures == 0 ? 0 : 1;
}
