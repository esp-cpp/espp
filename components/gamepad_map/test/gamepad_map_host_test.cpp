// Host-buildable unit tests for the gamepad_map component (GamepadState,
// GamepadMap, the hid-rp conversion, RumbleOptions and GamepadProfiles,
// including the version 1 -> 2 file migration). Build & run from the
// repository root (third-party headers are -isystem so their own warnings do
// not fail the build):
//
//   g++ -std=c++23 -Wall -Wextra -Werror -Wno-comment \
//       -I components/gamepad_map/include -I components/persistent_data/include \
//       -I components/serialization/include -I components/base_component/include \
//       -I components/logger/include -I components/format/include \
//       -isystem components/serialization/detail/alpaca/include \
//       -isystem components/format/detail/fmt/include \
//       -isystem components/hid-rp/include -isystem components/hid-rp/detail/hid-rp/hid-rp \
//       components/gamepad_map/test/gamepad_map_host_test.cpp components/logger/src/logger.cpp \
//       -o gamepad_map_host_test
//   ./gamepad_map_host_test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "gamepad_map.hpp"
#include "gamepad_profiles.hpp"
#include "gamepad_state_formatters.hpp"
#include "gamepad_state_hid_rp.hpp"
#include "rumble_options.hpp"
#include "rumble_state.hpp"

static int failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      failures++;                                                                                  \
    }                                                                                              \
  } while (0)

using espp::GamepadMap;
using espp::GamepadMapRule;
using espp::GamepadState;
using B = GamepadState::Button;
using A = GamepadState::Axis;

static GamepadState state(std::initializer_list<B> pressed,
                          const std::array<int16_t, 6> &axes = {}) {
  GamepadState s;
  for (auto b : pressed)
    s.set(b, true);
  s.axes = axes;
  return s;
}

static void test_state() {
  GamepadState s;
  s.set(B::East, true);
  s.set(B::Paddle4, true);
  CHECK(s.pressed(B::East) && s.pressed(B::Paddle4) && !s.pressed(B::South));
  s.set(B::East, false);
  CHECK(!s.pressed(B::East));
  s.set(B::Count, true); // ignored
  CHECK(s.buttons == (1u << static_cast<int>(B::Paddle4)));
  s.set(A::LeftX, -40000);
  CHECK(s.axis(A::LeftX) == -32767);
  s.set(A::RightTrigger, -5);
  CHECK(s.axis(A::RightTrigger) == 0); // triggers do not go negative
  CHECK(std::string(GamepadState::name(B::South)) == "south");
  CHECK(std::string(GamepadState::name(A::RightTrigger)) == "right_trigger");
  GamepadState f;
  f.set(B::South, true);
  f.set(B::Up, true);
  f.set(A::LeftY, -32767);
  f.set(A::RightTrigger, 32767);
  CHECK(fmt::format("{}", f) ==
        "{buttons: [south, up], lx: 0, ly: -32767, rx: 0, ry: 0, lt: 0, rt: 32767}");
}

static void test_identity() {
  GamepadMap map;
  CHECK(map.is_identity() && map.valid());
  const auto in = state({B::South, B::Up, B::Home}, {100, -200, 300, -400, 500, 600});
  CHECK(map.apply(in) == in);
}

static void test_swap_and_disable() {
  GamepadMap map;
  map.swap(B::South, B::East).add(GamepadMapRule::disable(B::Capture));
  CHECK(map.valid());
  auto out = map.apply(state({B::South, B::Capture, B::Start}));
  CHECK(out.pressed(B::East) && !out.pressed(B::South));
  CHECK(!out.pressed(B::Capture));
  CHECK(out.pressed(B::Start)); // untouched controls pass through
  out = map.apply(state({B::East}));
  CHECK(out.pressed(B::South) && !out.pressed(B::East));
  out = map.apply(state({B::South, B::East}));
  CHECK(out.pressed(B::South) && out.pressed(B::East));
}

static void test_keep_source_and_or() {
  GamepadMap map;
  // a paddle presses south too, keeping itself; L1 also presses south (moved)
  map.add(GamepadMapRule::button(B::Paddle1, B::South, true))
      .add(GamepadMapRule::button(B::L1, B::South));
  auto out = map.apply(state({B::Paddle1}));
  CHECK(out.pressed(B::Paddle1) && out.pressed(B::South));
  out = map.apply(state({B::L1}));
  CHECK(out.pressed(B::South) && !out.pressed(B::L1));
  out = map.apply(state({B::South}));
  CHECK(out.pressed(B::South)); // south is a target, not a source: still passes
}

static void test_axis_to_button() {
  GamepadMap map;
  map.left_stick_to_dpad(16384, /*keep_stick=*/false).triggers_to_buttons(13107, true);
  CHECK(map.valid());
  auto out = map.apply(state({}, {-20000, 20000, 0, 0, 13107, 13106}));
  CHECK(out.pressed(B::Left) && !out.pressed(B::Right));
  CHECK(out.pressed(B::Down) && !out.pressed(B::Up));
  CHECK(out.axis(A::LeftX) == 0 && out.axis(A::LeftY) == 0); // the stick was moved
  CHECK(out.pressed(B::L2) && !out.pressed(B::R2));          // >= threshold presses
  CHECK(out.axis(A::LeftTrigger) == 13107);                  // kept
  out = map.apply(state({}, {16383, -16384, 0, 0, 0, 0}));
  CHECK(!out.pressed(B::Right) && out.pressed(B::Up));
}

static void test_button_to_axis() {
  GamepadMap map;
  map.dpad_to_left_stick(/*keep_dpad=*/false).buttons_to_triggers(true);
  auto out = map.apply(state({B::Left, B::Down, B::R2}, {1000, 0, 0, 0, 0, 0}));
  // the d-pad adds to the stick, which still passes through, then clamps
  CHECK(out.axis(A::LeftX) == -32767 + 1000);
  CHECK(out.axis(A::LeftY) == 32767);
  CHECK(!out.pressed(B::Left) && !out.pressed(B::Down));
  CHECK(out.axis(A::RightTrigger) == 32767 && out.pressed(B::R2));
  out = map.apply(state({B::Left, B::Right}));
  CHECK(out.axis(A::LeftX) == 0); // opposite directions cancel
}

static void test_axis_to_axis() {
  GamepadMap map;
  map.invert(A::LeftY).swap_sticks();
  // invert(LeftY) moves LeftY to itself inverted; swap_sticks then moves LeftY
  // to RightY as well: both contribute to their targets
  auto out = map.apply(state({}, {100, 200, 300, 400, 0, 0}));
  CHECK(out.axis(A::LeftX) == 300 && out.axis(A::RightX) == 100);
  CHECK(out.axis(A::RightY) == 200);
  CHECK(out.axis(A::LeftY) == 400 - 200);

  GamepadMap trig;
  trig.add(GamepadMapRule::axis(A::RightY, A::RightTrigger, -100)); // stick up = trigger
  out = trig.apply(state({}, {0, 0, 0, -32767, 0, 0}));
  CHECK(out.axis(A::RightTrigger) == 32767 && out.axis(A::RightY) == 0);
  out = trig.apply(state({}, {0, 0, 0, 32767, 0, 0}));
  CHECK(out.axis(A::RightTrigger) == 0); // clamped at released

  // an inverted trigger reads from full instead of never moving
  GamepadMap inv_trigger;
  inv_trigger.invert(A::LeftTrigger);
  CHECK(inv_trigger.apply(state({}, {0, 0, 0, 0, 0, 0})).axis(A::LeftTrigger) == 32767);
  CHECK(inv_trigger.apply(state({}, {0, 0, 0, 0, 32767, 0})).axis(A::LeftTrigger) == 0);
  CHECK(inv_trigger.apply(state({}, {0, 0, 0, 0, 1000, 0})).axis(A::LeftTrigger) == 31767);
  // ... while a trigger inverted onto a stick goes negative
  GamepadMap trig_to_stick;
  trig_to_stick.add(GamepadMapRule::axis(A::RightTrigger, A::LeftY, -100));
  CHECK(trig_to_stick.apply(state({}, {0, 0, 0, 0, 0, 32767})).axis(A::LeftY) == -32767);

  GamepadMap gone;
  gone.add(GamepadMapRule::disable(A::LeftTrigger));
  CHECK(gone.apply(state({}, {0, 0, 0, 0, 9000, 9000})).axis(A::LeftTrigger) == 0);
}

static void test_deadzone() {
  GamepadMap map;
  map.left_deadzone_percent = 10;
  auto out = map.apply(state({}, {3000, -1000, 3000, 0, 0, 0}));
  CHECK(out.axis(A::LeftX) == 0 && out.axis(A::LeftY) == 0);
  CHECK(out.axis(A::RightX) == 3000); // only the left stick
  out = map.apply(state({}, {32767, 0, 0, 0, 0, 0}));
  CHECK(out.axis(A::LeftX) == 32767); // full deflection still full
  out = map.apply(state({}, {-20000, 0, 0, 0, 0, 0}));
  CHECK(out.axis(A::LeftX) < -18000 && out.axis(A::LeftX) > -20000); // rescaled
  // a square-gate corner (both axes full) stays a full diagonal
  map.left_deadzone_percent = 1;
  out = map.apply(state({}, {32767, 32767, 0, 0, 0, 0}));
  CHECK(out.axis(A::LeftX) == 32767 && out.axis(A::LeftY) == 32767);
}

static void test_validation() {
  std::string why;
  GamepadMap map;
  map.add({GamepadMapRule::Kind::ButtonToButton, 40, 0, 0, 0});
  CHECK(!map.valid(&why) && why.find("source") != std::string::npos);
  // an invalid rule is skipped by apply() instead of reading out of range
  CHECK(map.apply(state({B::South})).pressed(B::South));
  map.rules = {GamepadMapRule::axis_to_button(A::LeftX, 0, B::Left)};
  CHECK(!map.valid(&why));
  map.rules = {GamepadMapRule::axis(A::LeftX, A::RightX, 401)};
  CHECK(!map.valid(&why));
  map.rules = {{GamepadMapRule::Kind::ButtonToAxis, 0, 6, 0, 0}}; // axis 6 does not exist
  CHECK(!map.valid(&why));
  map.rules = {{GamepadMapRule::Kind::ButtonToButton, 0, 1, 0x80, 0}};
  CHECK(!map.valid(&why)); // unknown flag
  map.rules = {GamepadMapRule::axis_to_button(A::LeftTrigger, -100, B::L2)};
  CHECK(!map.valid(&why)); // a trigger never goes below 0
  map.rules = {GamepadMapRule::axis_to_button(A::LeftTrigger, 100, B::L2)};
  CHECK(map.valid());
  map.rules = {{GamepadMapRule::Kind::ButtonToButton, 0, GamepadMapRule::NONE,
                GamepadMapRule::KEEP_SOURCE, 0}};
  CHECK(!map.valid(&why)); // keeping the source of a rule with no target does nothing
  map.rules.assign(GamepadMap::MAX_RULES + 1, GamepadMapRule::button(B::South, B::East));
  CHECK(!map.valid(&why) && why == "too many rules");
  map.rules.clear();
  map.right_deadzone_percent = 91;
  CHECK(!map.valid(&why));
}

static void test_aliasing() {
  GamepadMap map;
  map.swap(B::South, B::East);
  auto s = state({B::South});
  map.apply(s, s);
  CHECK(s.pressed(B::East) && !s.pressed(B::South));
}

// BSP-D8 (ShanWan BM-769) in its Android mode, as reported over BLE: the
// decoder + the conversion give a usable state, and a map applies on top.
static void test_hid_rp_bridge() {
  const char *hex =
      "050c0901a1018503751095011501269c0219012a9c028100c005010905a10185040901a1000930093109320935"
      "150026ff00750895048102c00939150025073500463b016514750495018142750495018101050919012910150025"
      "017501951081020502150026ff0009c409c5950275088102750895018101c0";
  std::vector<uint8_t> desc;
  for (const char *p = hex; p[0] && p[1]; p += 2)
    desc.push_back(static_cast<uint8_t>(std::stoi(std::string(p, 2), nullptr, 16)));
  auto map = espp::hid_rp::ReportMap::parse(desc);
  CHECK(map.has_value());
  if (!map)
    return;
  espp::hid_rp::GamepadDecoder decoder(std::move(*map));
  decoder.set_combine_controls(false);
  // A pressed, hat up, right trigger full, left stick full right
  const std::vector<uint8_t> pad{0xff, 0x80, 0x80, 0x80, 0x00, 0x01, 0x00, 0xff, 0x00, 0x00};
  espp::hid_rp::GamepadReport r;
  CHECK(decoder.update(4, pad, r));
  auto s = espp::to_gamepad_state(r);
  CHECK(s.pressed(B::South) && s.pressed(B::Up));
  CHECK(s.axis(A::RightTrigger) > 32000 && s.axis(A::LeftX) > 32000);
  CHECK(!s.pressed(B::R2) && !s.pressed(B::Right)); // nothing combined

  // a Nintendo-style host: A is east
  GamepadMap nintendo;
  nintendo.swap(B::South, B::East).triggers_to_buttons();
  s = nintendo.apply(s);
  CHECK(s.pressed(B::East) && !s.pressed(B::South) && s.pressed(B::R2));
}

static void test_profiles() {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "gamepad_map_host_test";
  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path file = dir / "profiles.bin";
  std::error_code ec;

  GamepadMap legacy; // e.g. migrated from an application's old settings
  legacy.invert(A::LeftY);
  {
    espp::GamepadProfiles profiles({.file_path = file, .default_map = legacy});
    CHECK(profiles.default_map() == legacy); // no file yet: the configured default
    CHECK(!fs::exists(file));                // ... and nothing written until a change
    CHECK(profiles.map_for(0x1949, 0x0402) == legacy);

    GamepadMap bsp;
    bsp.swap(B::South, B::East);
    CHECK(profiles.set_profile({.vid = 0x1949, .pid = 0x0402, .name = "BSP-D8", .map = bsp}, ec));
    CHECK(!ec && fs::exists(file));
    CHECK(profiles.map_for(0x1949, 0x0402) == bsp);
    CHECK(profiles.map_for(0x045e, 0x0b13) == legacy);

    GamepadMap named;
    named.left_deadzone_percent = 12;
    CHECK(profiles.set_profile({.vid = 0, .pid = 0, .name = "NoPnP", .map = named}, ec));
    CHECK(profiles.map_for(0, 0, "NoPnP") == named);
    CHECK(profiles.map_for(0, 0, "Other") == legacy);
    CHECK(!profiles.set_profile({.vid = 0, .pid = 0, .name = "", .map = named}, ec));
    CHECK(ec == std::errc::invalid_argument);

    GamepadMap bad;
    bad.right_deadzone_percent = 99;
    CHECK(!profiles.set_profile({.vid = 1, .pid = 2, .name = "x", .map = bad}, ec));
    CHECK(ec == std::errc::invalid_argument);
    CHECK(!profiles.set_default_map(bad, ec));

    // replacing keeps one entry per controller
    bsp.rules.clear();
    CHECK(profiles.set_profile({.vid = 0x1949, .pid = 0x0402, .name = "BSP-D8", .map = bsp}, ec));
    CHECK(profiles.profiles().size() == 2);
  }
  {
    // reloaded from the file
    espp::GamepadProfiles profiles({.file_path = file, .default_map = GamepadMap{}});
    CHECK(profiles.default_map() == legacy); // the stored default, not the configured one
    CHECK(profiles.profiles().size() == 2);
    CHECK(profiles.map_for(0x1949, 0x0402).rules.empty());
    CHECK(profiles.erase_profile(0x1949, 0x0402, "", ec) && !ec);
    CHECK(!profiles.erase_profile(0x1949, 0x0402, "", ec) && !ec); // already gone
    CHECK(profiles.map_for(0x1949, 0x0402) == legacy);
  }
  {
    espp::GamepadProfiles profiles({.file_path = file, .max_profiles = 1});
    CHECK(!profiles.load_error());
    CHECK(profiles.profile_count() == 1 && profiles.has_profile(0, 0, "NoPnP"));
    CHECK(!profiles.has_profile(0x1949, 0x0402));
    CHECK(profiles.profiles().size() == 1); // NoPnP
    CHECK(!profiles.set_profile({.vid = 5, .pid = 6, .name = "full", .map = {}}, ec));
    CHECK(ec == std::errc::no_buffer_space);
    CHECK(profiles.reset(ec) && profiles.profiles().empty() &&
          profiles.default_map().is_identity());
  }
  {
    // names are stored cut to MAX_NAME_LENGTH and still match by name
    espp::GamepadProfiles profiles({.file_path = file});
    const std::string long_name(100, 'n');
    CHECK(profiles.set_profile({.vid = 0, .pid = 0, .name = long_name, .map = {}}, ec));
    CHECK(profiles.has_profile(0, 0, long_name));
    CHECK(profiles.profiles().back().name.size() == espp::GamepadProfile::MAX_NAME_LENGTH);
  }
  {
    // a file that cannot be read is set aside, never overwritten
    {
      std::ofstream garbage(file, std::ios::binary | std::ios::trunc);
      garbage << "definitely not a profile store";
    }
    espp::GamepadProfiles profiles({.file_path = file});
    CHECK(profiles.load_error());
    CHECK(fs::exists(fs::path(file.string() + ".unreadable")));
    CHECK(!fs::exists(file));
    CHECK(profiles.profiles().empty());
    CHECK(profiles.set_default_map(GamepadMap{}, ec) && fs::exists(file));
    CHECK(fs::exists(fs::path(file.string() + ".unreadable"))); // still there
  }
  {
    // stored entries are sanitized: an invalid rule is dropped (the rest of
    // its map kept), a profile nothing can match is dropped
    espp::GamepadProfiles::Data d;
    GamepadMap m;
    m.add(GamepadMapRule::button(B::South, B::East));
    m.add({GamepadMapRule::Kind::ButtonToButton, 99, 0, 0, 0}); // invalid source
    d.profiles.push_back({.vid = 1, .pid = 2, .name = "ok", .map = m});
    d.profiles.push_back({.vid = 0, .pid = 0, .name = "", .map = {}}); // unmatchable
    espp::GamepadProfiles::Storage raw({.file_path = file, .auto_load = false});
    CHECK(raw.save(d, ec));
    espp::GamepadProfiles profiles({.file_path = file});
    CHECK(!profiles.load_error());
    CHECK(profiles.profile_count() == 1);
    const auto kept = profiles.profile(1, 2);
    CHECK(kept.has_value() && kept->map.rules.size() == 1 && kept->map.valid());
  }
  fs::remove_all(dir);
}

static void test_rumble_state() {
  using espp::RumbleState;
  using M = RumbleState::Motor;
  RumbleState r;
  CHECK(!r.any());
  r.set(M::Strong, 200);
  r.set(M::RightTrigger, 10);
  r.set(M::Count, 99); // ignored
  CHECK(r.any() && r.level(M::Strong) == 200 && r.level(M::Weak) == 0 &&
        r.level(M::RightTrigger) == 10 && r.level(M::Count) == 0);
  // to the encoder's levels, motor by motor
  const auto l = espp::to_rumble_levels(r);
  CHECK(l.strong == 200 && l.weak == 0 && l.left_trigger == 0 && l.right_trigger == 10);
  // a Switch host's left / right motors are strong / weak
  const auto s = espp::to_rumble_state(espp::hid_rp::SwitchRumble{.left = 117, .right = 40});
  CHECK(s.level(M::Strong) == 117 && s.level(M::Weak) == 40 && s.level(M::LeftTrigger) == 0);
  r.clear();
  CHECK(!r.any() && r == RumbleState{});
}

static void test_rumble_options() {
  using espp::RumbleOptions;
  using espp::RumbleState;
  using M = RumbleState::Motor;
  constexpr auto make = [](uint8_t strong, uint8_t weak, uint8_t lt, uint8_t rt) {
    RumbleState r;
    r.set(M::Strong, strong);
    r.set(M::Weak, weak);
    r.set(M::LeftTrigger, lt);
    r.set(M::RightTrigger, rt);
    return r;
  };
  constexpr RumbleState sample = make(200, 10, 5, 7);
  static_assert(RumbleOptions{}.apply(sample) == sample); // neutral, at compile time

  // 100 % with no flags is the exact identity for every level on every motor
  bool identity = true;
  for (int level = 0; level <= 255; ++level) {
    const auto l = static_cast<uint8_t>(level);
    identity = identity && RumbleOptions{}.apply(make(l, l, l, l)) == make(l, l, l, l);
  }
  CHECK(identity);

  // the parity table (the console's applyRumble() is tested against the same rows)
  struct Row {
    RumbleOptions o;
    RumbleState in, out;
  };
  constexpr uint8_t OFF = RumbleOptions::RUMBLE_OFF;
  constexpr uint8_t SWAP = RumbleOptions::SWAP_MOTORS;
  const Row rows[] = {
      {{0, 100}, make(200, 10, 5, 7), make(200, 10, 5, 7)},
      {{OFF, 100}, make(200, 10, 5, 7), make(0, 0, 0, 0)},
      {{0, 0}, make(200, 10, 5, 7), make(0, 0, 0, 0)},
      {{0, 50}, make(255, 1, 3, 0), make(128, 1, 2, 0)},
      {{0, 1}, make(255, 1, 0, 0), make(3, 0, 0, 0)},
      {{0, 49}, make(1, 0, 0, 0), make(0, 0, 0, 0)},
      {{0, 150}, make(100, 200, 128, 0), make(150, 255, 192, 0)},
      {{0, 200}, make(127, 128, 255, 100), make(254, 255, 255, 200)},
      {{SWAP, 100}, make(200, 10, 5, 7), make(10, 200, 5, 7)},
      {{SWAP, 50}, make(200, 10, 5, 7), make(5, 100, 3, 4)},
      {{0xFC, 100}, make(200, 10, 5, 7), make(200, 10, 5, 7)}, // unknown bits only
      {{0, 250}, make(100, 0, 0, 0), make(200, 0, 0, 0)},      // invalid: behaves as 200
  };
  for (const auto &row : rows) {
    if (row.o.apply(row.in) != row.out) {
      std::printf("FAIL rumble parity row {0x%02x, %u}\n", row.o.flags, row.o.strength_percent);
      failures++;
    }
  }

  // a stop stays a stop for every option value
  bool stop_safe = true;
  for (uint8_t flags = 0; flags <= 3; ++flags)
    for (int pct = 0; pct <= 200; ++pct)
      stop_safe = stop_safe && RumbleOptions{flags, static_cast<uint8_t>(pct)}.apply(
                                   RumbleState{}) == RumbleState{};
  CHECK(stop_safe);

  // monotonic in the level and in the strength
  bool monotonic = true;
  for (int pct = 0; pct <= 200; ++pct) {
    const RumbleOptions o{0, static_cast<uint8_t>(pct)};
    for (int level = 1; level <= 255; ++level)
      monotonic =
          monotonic && o.apply(make(static_cast<uint8_t>(level), 0, 0, 0)).level(M::Strong) >=
                           o.apply(make(static_cast<uint8_t>(level - 1), 0, 0, 0)).level(M::Strong);
  }
  for (int level = 0; level <= 255; ++level)
    for (int pct = 1; pct <= 200; ++pct)
      monotonic = monotonic && RumbleOptions{0, static_cast<uint8_t>(pct)}
                                       .apply(make(static_cast<uint8_t>(level), 0, 0, 0))
                                       .level(M::Strong) >=
                                   RumbleOptions{0, static_cast<uint8_t>(pct - 1)}
                                       .apply(make(static_cast<uint8_t>(level), 0, 0, 0))
                                       .level(M::Strong);
  CHECK(monotonic);

  // swap twice at 100 % is the identity; swap and scaling commute
  const RumbleOptions swap{SWAP, 100};
  CHECK(swap.apply(swap.apply(sample)) == sample);
  CHECK((RumbleOptions{SWAP, 150}.apply(sample)) ==
        swap.apply(RumbleOptions{0, 150}.apply(sample)));
  // off keeps the stored strength (turning it back on restores it)
  const RumbleOptions off{OFF, 150};
  CHECK(off.is_off() && off.strength_percent == 150);

  // validation
  std::string why;
  CHECK((RumbleOptions{0x03, 200}.valid()));
  CHECK(!(RumbleOptions{0x04, 100}.valid(&why)) && why == "unknown rumble flags");
  CHECK(!(RumbleOptions{0, 201}.valid(&why)) && why == "strength out of range (0..200 %)");
  // neutral: unknown bits ignored
  CHECK(RumbleOptions{}.is_neutral() && (RumbleOptions{0xFC, 100}.is_neutral()));
  CHECK(!(RumbleOptions{SWAP, 100}.is_neutral()) && !(RumbleOptions{0, 99}.is_neutral()));
}

static std::vector<uint8_t> read_file(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

static uint32_t file_version(const std::filesystem::path &path) {
  const auto bytes = read_file(path);
  return bytes.size() < 8 ? 0
                          : static_cast<uint32_t>(bytes[4] | (bytes[5] << 8) | (bytes[6] << 16) |
                                                  (bytes[7] << 24));
}

static void test_profiles_rumble() {
  namespace fs = std::filesystem;
  using espp::RumbleOptions;
  const fs::path dir = fs::temp_directory_path() / "gamepad_map_host_test_rumble";
  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path file = dir / "profiles.bin";
  std::error_code ec;
  const RumbleOptions swap150{.flags = RumbleOptions::SWAP_MOTORS, .strength_percent = 150};
  const RumbleOptions off{.flags = RumbleOptions::RUMBLE_OFF, .strength_percent = 100};
  const RumbleOptions half{.flags = 0, .strength_percent = 50};
  GamepadMap bsp;
  bsp.swap(B::South, B::East);
  {
    // Config::default_rumble applies with no file, and nothing is written
    espp::GamepadProfiles profiles({.file_path = file, .default_rumble = half});
    CHECK(profiles.default_rumble() == half && !fs::exists(file));
    CHECK(profiles.rumble_for(1, 2) == half);
  }
  {
    // an invalid configured default becomes neutral
    espp::GamepadProfiles profiles({.file_path = file, .default_rumble = {0, 250}});
    CHECK(profiles.default_rumble() == RumbleOptions{});
  }
  {
    espp::GamepadProfiles profiles({.file_path = file});
    CHECK(profiles.set_profile(
        {.vid = 0x1949, .pid = 0x0402, .name = "BSP-D8", .map = bsp, .rumble = swap150}, ec));
    CHECK(profiles.rumble_for(0x1949, 0x0402) == swap150);
    CHECK(profiles.rumble_for(0x045e, 0x0b13) == profiles.default_rumble());
    CHECK(profiles.set_default_rumble(half, ec) && profiles.default_rumble() == half);
    GamepadMap inverted;
    inverted.invert(A::LeftY);
    CHECK(profiles.set_defaults(inverted, off, ec));
    CHECK(profiles.default_map() == inverted && profiles.default_rumble() == off);
    CHECK(profiles.set_default_map(GamepadMap{}, ec) && profiles.default_rumble() == off);
  }
  {
    // reloaded: everything persisted
    espp::GamepadProfiles profiles({.file_path = file});
    CHECK(!profiles.load_error());
    CHECK(profiles.rumble_for(0x1949, 0x0402) == swap150);
    CHECK(profiles.default_rumble() == off && profiles.default_map().is_identity());

    // set_profile_map keeps a stored profile's rumble, changes its map
    GamepadMap other;
    other.left_deadzone_percent = 7;
    CHECK(profiles.set_profile_map(0x1949, 0x0402, "BSP-D8", other, ec));
    CHECK(profiles.map_for(0x1949, 0x0402) == other &&
          profiles.rumble_for(0x1949, 0x0402) == swap150);
    // ... and a new profile starts from the default rumble
    CHECK(profiles.set_profile_map(0x0001, 0x0002, "new", other, ec));
    CHECK(profiles.rumble_for(0x0001, 0x0002) == off);
    CHECK(!profiles.set_profile_map(0, 0, "", other, ec) && ec == std::errc::invalid_argument);

    // set_profile replaces the whole profile, rumble included
    CHECK(profiles.set_profile({.vid = 0x0001, .pid = 0x0002, .name = "new", .map = other}, ec));
    CHECK(profiles.rumble_for(0x0001, 0x0002) == RumbleOptions{});

    // invalid options are refused and nothing is written
    const auto before = read_file(file);
    CHECK(!profiles.set_profile({.vid = 3, .pid = 4, .name = "x", .map = {}, .rumble = {0x04, 100}},
                                ec) &&
          ec == std::errc::invalid_argument);
    CHECK(!profiles.set_default_rumble({0, 201}, ec) && ec == std::errc::invalid_argument);
    CHECK(!profiles.set_defaults(GamepadMap{}, {0, 201}, ec) && ec == std::errc::invalid_argument);
    CHECK(read_file(file) == before);

    // erase: the controller falls back to the default; reset: default neutral
    CHECK(profiles.erase_profile(0x1949, 0x0402, "", ec));
    CHECK(profiles.rumble_for(0x1949, 0x0402) == off);
    CHECK(profiles.reset(ec) && profiles.default_rumble() == RumbleOptions{});
  }
  {
    // full store: set_profile_map reports no_buffer_space
    espp::GamepadProfiles profiles({.file_path = file, .max_profiles = 1});
    CHECK(profiles.set_profile_map(1, 1, "a", GamepadMap{}, ec));
    CHECK(!profiles.set_profile_map(2, 2, "b", GamepadMap{}, ec) &&
          ec == std::errc::no_buffer_space);
  }
  {
    // stored options are sanitized: unknown bits cleared, strength clamped, the profile kept
    espp::GamepadProfiles::Data d;
    d.default_rumble = {0xFC, 250};
    d.profiles.push_back({.vid = 1, .pid = 2, .name = "ok", .map = {}, .rumble = {0x06, 255}});
    espp::GamepadProfiles::Storage raw({.file_path = file, .auto_load = false});
    CHECK(raw.save(d, ec));
    espp::GamepadProfiles profiles({.file_path = file});
    CHECK(!profiles.load_error());
    CHECK((profiles.default_rumble() == RumbleOptions{0x00, 200}));
    CHECK(profiles.profile_count() == 1 && (profiles.rumble_for(1, 2) == RumbleOptions{0x02, 200}));
  }
  fs::remove_all(dir);
}

static void test_profiles_v1_migration() {
  namespace fs = std::filesystem;
  using espp::RumbleOptions;
  // A version 1 file as espp before rumble options wrote it (default map
  // invert(LeftY); 1949:0402 "BSP-D8" swap(South, East); 0:0 "NoPnP" with a
  // 12 % left dead zone), from GamepadProfiles::Storage::encode().
  static const uint8_t kV1File[] = {
      0x45, 0x53, 0x50, 0x44, 0x01, 0x00, 0x00, 0x00, 0x3b, 0xa1, 0xd1, 0x4e, 0x43, 0x00, 0x00,
      0x00, 0x4a, 0x32, 0xe8, 0x17, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x01, 0x01, 0x00,
      0x9c, 0xff, 0x02, 0x00, 0x00, 0x00, 0x49, 0x19, 0x02, 0x04, 0x06, 0x00, 0x00, 0x00, 0x42,
      0x53, 0x50, 0x2d, 0x44, 0x38, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00,
      0x00, 0x4e, 0x6f, 0x50, 0x6e, 0x50, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  };
  // changing this means V1 files no longer load: GamepadProfileV1 /
  // GamepadProfilesV1 (and GamepadMap / GamepadMapRule) are frozen
  CHECK(espp::persistent_data::layout::fingerprint<espp::GamepadProfilesV1>() == 0x4ed1a13b);
  // a change to the V2 layout needs a V3
  CHECK(espp::GamepadProfiles::Storage::layout_fingerprint() == 0x64dff6d1);
  CHECK(espp::GamepadProfiles::Storage::CURRENT_VERSION == 2);

  const fs::path dir = fs::temp_directory_path() / "gamepad_map_host_test_v1";
  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path file = dir / "profiles.bin";
  {
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(kV1File), sizeof(kV1File));
  }
  std::error_code ec;
  GamepadMap legacy;
  legacy.invert(A::LeftY);
  GamepadMap bsp;
  bsp.swap(B::South, B::East);
  GamepadMap named;
  named.left_deadzone_percent = 12;
  espp::GamepadProfiles::Data loaded;
  {
    // the configured default rumble does not apply to a migrated file
    espp::GamepadProfiles profiles({.file_path = file, .default_rumble = {0, 50}});
    CHECK(!profiles.load_error());
    CHECK(profiles.default_map() == legacy);
    CHECK(profiles.map_for(0x1949, 0x0402) == bsp && profiles.map_for(0, 0, "NoPnP") == named);
    CHECK(profiles.default_rumble() == RumbleOptions{});
    CHECK(profiles.rumble_for(0x1949, 0x0402) == RumbleOptions{} &&
          profiles.rumble_for(0, 0, "NoPnP") == RumbleOptions{});
    CHECK(file_version(file) == 1); // not rewritten on load
    CHECK(profiles.set_default_rumble(RumbleOptions{}, ec));
    CHECK(file_version(file) == 2); // the first change writes version 2
    loaded = profiles.data();
  }
  {
    espp::GamepadProfiles profiles({.file_path = file});
    const auto again = profiles.data();
    CHECK(!profiles.load_error() && again.default_map == loaded.default_map &&
          again.default_rumble == loaded.default_rumble && again.profiles == loaded.profiles);
  }
  {
    // older firmware (V1 only) cannot read a version 2 file
    espp::PersistentData<espp::GamepadProfilesV1> old({.file_path = file, .auto_load = false});
    CHECK(!old.load(ec) && ec == espp::PersistentDataErrc::unknown_version);
  }
  fs::remove_all(dir);
}

int main() {
  test_state();
  test_identity();
  test_swap_and_disable();
  test_keep_source_and_or();
  test_axis_to_button();
  test_button_to_axis();
  test_axis_to_axis();
  test_deadzone();
  test_validation();
  test_aliasing();
  test_hid_rp_bridge();
  test_rumble_state();
  test_rumble_options();
  test_profiles();
  test_profiles_rumble();
  test_profiles_v1_migration();
  if (failures == 0) {
    std::printf("gamepad_map_host_test: all checks passed\n");
    return 0;
  }
  std::printf("gamepad_map_host_test: %d failure(s)\n", failures);
  return 1;
}
