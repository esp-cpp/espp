// Host-buildable unit tests for the gamepad map wire protocol
// (include/detail/gamepad_map_protocol.hpp): golden encodings of every message
// plus decode round trips and rejection of malformed payloads. Build & run
// from the repository root:
//
//   g++ -std=c++23 -Wall -Wextra -Werror
//       -I components/gamepad_map/include -I components/stream_frame/include
//       components/gamepad_map/test/gamepad_map_protocol_host_test.cpp
//       -o gamepad_map_protocol_host_test
//   ./gamepad_map_protocol_host_test
//
// No ESP-IDF headers required.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "detail/gamepad_map_protocol.hpp"

namespace proto = espp::detail::gamepad_map_protocol;
using espp::GamepadMap;
using espp::GamepadMapRule;
using espp::GamepadState;
using B = GamepadState::Button;
using A = GamepadState::Axis;

static int failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                  \
      failures++;                                                                                  \
    }                                                                                              \
  } while (0)

using Bytes = std::vector<uint8_t>;

static void test_map_encoding() {
  GamepadMap m;
  m.left_deadzone_percent = 10;
  m.right_deadzone_percent = 5;
  m.add(GamepadMapRule::button(B::South, B::East))
      .add(GamepadMapRule::axis_to_button(A::LeftY, -16384, B::Up, true));
  const proto::Target t{.profile = true, .vid = 0x1949, .pid = 0x0402, .name = "BSP"};
  const Bytes p = proto::encode_set_map(t, m);
  const Bytes golden{
      0x01, 0x49, 0x19, 0x02, 0x04, 0x03, 'B', 'S', 'P', // target: profile 1949:0402 "BSP"
      10,   5,    2,                                     // dead zones, 2 rules
      0x00, 0x00, 0x01, 0x00, 0x00, 0x00,                // south -> east
      0x01, 0x01, 0x0e, 0x01, 0x00, 0xc0,                // left_y <= -16384 -> up, keep
  };
  CHECK(p == golden);
  const auto back = proto::decode_set_map(p);
  CHECK(back.has_value());
  if (back) {
    CHECK(back->first == t);
    CHECK(back->second == m);
  }
  // the default target is one byte
  const Bytes def = proto::encode_get_map({});
  CHECK(def == Bytes{0x00});
  CHECK(proto::decode_get_map(def).has_value() && !proto::decode_get_map(def)->profile);
  // malformed: unknown target kind, truncated rule, trailing bytes
  CHECK(!proto::decode_get_map(Bytes{0x02}));
  Bytes truncated = golden;
  truncated.pop_back();
  CHECK(!proto::decode_set_map(truncated));
  Bytes trailing = golden;
  trailing.push_back(0);
  CHECK(!proto::decode_set_map(trailing));
  // the layout decodes even when the content is invalid: valid() is the
  // service's job
  const Bytes bad_rule{0x00, 0, 0, 1, 0x07, 0x40, 0x00, 0x00, 0x00, 0x00};
  const auto invalid = proto::decode_set_map(bad_rule);
  CHECK(invalid.has_value() && !invalid->second.valid());

  // MAP: the target, the exists flag, the map; a host ignores appended fields
  Bytes reply = proto::encode_map_reply(t, false, m);
  CHECK(reply.size() == golden.size() + 1);
  CHECK(reply[9] == 0x00); // after the 9-byte target: exists = 0
  auto decoded = proto::decode_map_reply(reply);
  CHECK(decoded.has_value() && decoded->target == t && !decoded->exists && decoded->map == m);
  reply = proto::encode_map_reply({}, true, GamepadMap{});
  CHECK(reply == (Bytes{0x00, 0x01, 0, 0, 0}));
  reply.push_back(0xAA); // a later version's field
  decoded = proto::decode_map_reply(reply);
  CHECK(decoded.has_value() && decoded->exists && !decoded->target.profile);
}

static void test_info() {
  proto::Info info{
      .button_count = 22, .axis_count = 6, .max_rules = 64, .max_profiles = 16, .profile_count = 1};
  Bytes p = proto::encode_info(info);
  CHECK(p == (Bytes{22, 6, 64, 16, 1, 0x00}));
  auto back = proto::decode_info(p);
  CHECK(back.has_value() && *back == info);

  info.controller =
      proto::Controller{.vid = 0x045e, .pid = 0x0b13, .name = "Xbox", .has_profile = true};
  p = proto::encode_info(info);
  CHECK(p == (Bytes{22, 6, 64, 16, 1, 0x01, 0x5e, 0x04, 0x13, 0x0b, 4, 'X', 'b', 'o', 'x', 1}));
  back = proto::decode_info(p);
  CHECK(back.has_value() && *back == info);
  Bytes longer = p;
  longer.push_back(0x55); // device -> host: appended fields are ignored
  CHECK(proto::decode_info(longer).has_value() && *proto::decode_info(longer) == info);
  p.pop_back();
  CHECK(!proto::decode_info(p));
}

static void test_profiles() {
  const std::vector<proto::ProfileSummary> list{{0x1949, 0x0402, "BSP-D8", 4}, {0, 0, "NoPnP", 0}};
  const Bytes p = proto::encode_profiles(list);
  const Bytes golden{2, 0x49, 0x19, 0x02, 0x04, 6, 'B', 'S', 'P', '-', 'D', '8',
                     4, 0x00, 0x00, 0x00, 0x00, 5, 'N', 'o', 'P', 'n', 'P', 0};
  CHECK(p == golden);
  const auto back = proto::decode_profiles(p);
  CHECK(back.has_value() && *back == list);
  // capped: only what fits
  size_t encoded = 0;
  const Bytes capped = proto::encode_profiles(list, 1 + 12, &encoded);
  CHECK(encoded == 1 && capped.size() == 13 && capped[0] == 1);
}

static void test_input_and_stream() {
  proto::Input in;
  in.connected = true;
  in.changed = true;
  in.raw.set(B::South, true);
  in.raw.set(A::LeftX, -32767);
  in.mapped.set(B::East, true);
  in.mapped.set(A::RightTrigger, 32767);
  const Bytes p = proto::encode_input(in);
  CHECK(p.size() == 1 + 2 * proto::kStateSize);
  CHECK(p[0] == 0x03);
  CHECK(p[1] == 0x01 && p[2] == 0 && p[3] == 0 && p[4] == 0); // raw buttons: south
  CHECK(p[5] == 0x01 && p[6] == 0x80);                        // raw left_x = -32767
  CHECK(p[17] == 0x02);                                       // mapped buttons: east
  CHECK(p[31] == 0xff && p[32] == 0x7f);                      // mapped right_trigger
  const auto back = proto::decode_input(p);
  CHECK(back.has_value());
  if (back) {
    CHECK(back->connected && back->changed);
    CHECK(back->raw == in.raw && back->mapped == in.mapped);
  }

  const Bytes s = proto::encode_set_stream(true, 50);
  CHECK(s == (Bytes{1, 50, 0}));
  const auto req = proto::decode_set_stream(s);
  CHECK(req.has_value() && req->enable && req->period_ms == 50);
  CHECK(!proto::decode_set_stream(Bytes{2, 50, 0})); // enable must be 0 / 1
  CHECK(!proto::decode_set_stream(Bytes{1, 50}));
}

static void test_erase_error_frame() {
  const Bytes e = proto::encode_erase_profile(0x1949, 0x0402, "BSP");
  CHECK(e == (Bytes{0x49, 0x19, 0x02, 0x04, 3, 'B', 'S', 'P'}));
  const auto t = proto::decode_erase_profile(e);
  CHECK(t.has_value() && t->profile && t->vid == 0x1949 && t->name == "BSP");

  const Bytes err = proto::encode_error(0x03, 22, "bad");
  CHECK(err == (Bytes{0x03, 22, 0, 0, 0, 'b', 'a', 'd'}));

  // replies carry the reply flag, requests do not; the frame decodes back
  const auto frame = proto::build_frame(proto::Type::Ok, proto::encode_ok(0x03), proto::kModule, 7);
  espp::stream_frame::StreamParser parser;
  const auto frames = parser.feed(frame);
  CHECK(frames.size() == 1);
  if (frames.size() == 1) {
    CHECK(frames[0].is_reply());
    CHECK(frames[0].module == 11);
    CHECK(frames[0].type == 0x83);
    CHECK(frames[0].correlation == std::optional<uint16_t>(7));
  }
  CHECK(!proto::is_reply(proto::Type::SetMap) && proto::is_reply(proto::Type::Input));
}

int main() {
  test_map_encoding();
  test_info();
  test_profiles();
  test_input_and_stream();
  test_erase_error_frame();
  if (failures == 0) {
    std::printf("gamepad_map_protocol_host_test: all checks passed\n");
    return 0;
  }
  std::printf("gamepad_map_protocol_host_test: %d failure(s)\n", failures);
  return 1;
}
