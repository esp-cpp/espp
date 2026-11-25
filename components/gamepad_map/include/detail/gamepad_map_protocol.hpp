#pragma once

// Wire protocol of espp::GamepadMapService: view and edit the gamepad maps of
// an espp::GamepadProfiles store, and watch a controller's input before and
// after mapping, over the espp stream_frame codec. Routed by an
// espp::Dispatcher on module 11 by default (`espp.gamepad_map` v1 through
// discovery).
//
// This header is deliberately host-buildable (stream_frame.hpp, gamepad_map.hpp,
// rumble_options.hpp and the standard library only) so the codec is unit-tested
// on the host (components/gamepad_map/test/gamepad_map_protocol_host_test.cpp)
// and so host tools can reuse it. All multi-byte fields are little-endian; a `str` is
// [len u8][utf8 bytes].
//
// Shared encodings:
//   target  [kind u8]                 0 = the default map (nothing follows)
//           [kind u8 = 1][vid u16][pid u16][name str]
//                                     1 = the profile for that controller type
//                                         (VID and PID 0: matched by name)
//   map     [left_deadzone u8][right_deadzone u8][rule_count u8]
//           {[kind u8][source u8][target u8][flags u8][param i16]} x rule_count
//           (GamepadMapRule field for field; 6 bytes per rule)
//   state   [buttons u32][left_x i16][left_y i16][right_x i16][right_y i16]
//           [left_trigger i16][right_trigger i16]          (16 bytes)
//   rumble  [flags u8][strength_percent u8]   (2 bytes; RumbleOptions field for field)
//           flags bit0 = RUMBLE_OFF, bit1 = SWAP_MOTORS, bits 2-7 reserved;
//           strength 0..200; neutral = 00 64
//
// Requests (host -> device, reply flag clear):
//   0x01 GET_INFO       (no payload)
//   0x02 GET_MAP        [target]
//   0x03 SET_MAP        [target][map]  or  [target][map][rumble]
//                       validated, then saved; a profile target adds or
//                       replaces that profile. Exactly 0 or 2 bytes may follow
//                       the map (anything else is malformed). With the rumble
//                       block, the map and the options are saved together;
//                       without it the stored options are kept (a new profile
//                       starts from the default rumble options)
//   0x04 ERASE_PROFILE  [vid u16][pid u16][name str]
//                       the profile's rumble options go with it (the controller
//                       falls back to the default rumble options)
//   0x05 LIST_PROFILES  (no payload)
//   0x06 SET_STREAM     [enable u8][period_ms u16]
//                       INPUT events every period while enabled; the device
//                       clamps the period to its minimum and stops by itself
//                       when no SET_STREAM enable renews the stream within its
//                       lease (GamepadMapService::Config::stream_lease), so a
//                       closed page does not leave it streaming
//   0x07 RESET_ALL      (no payload)    every profile erased, default = identity,
//                                       default rumble options = neutral
//   0x08 TEST_RUMBLE    [rumble]        play a short test pattern on the
//                                       connected controller through these
//                                       (unsaved) rumble options; nothing is
//                                       saved. Only when INFO flags bit2 is
//                                       set; otherwise refused (not supported)
// Replies / events (device -> host, high bit set = frame reply flag):
//   0x81 INFO      [button_count u8][axis_count u8][max_rules u8]
//                  [max_profiles u8][profile_count u8][flags u8]
//                  flags bit0 = a controller is connected, then:
//                  [vid u16][pid u16][name str][has_profile u8]
//                  flags bit1 = the device stores rumble options per map (MAP
//                  carries them and SET_MAP accepts them); adds no bytes
//                  flags bit2 = the device takes TEST_RUMBLE; adds no bytes
//                  Answers GET_INFO, and is also sent unsolicited (no
//                  correlation id) when the INPUT stream starts and when the
//                  connected controller changes while it is on.
//   0x82 MAP       [target][flags u8][map]  (GET_MAP; the target as requested,
//                                          the profile's stored name if it
//                                          exists; flags bit0 = it exists. A
//                                          profile that does not exist answers
//                                          with the map its controller uses
//                                          today, the default map, exists 0)
//                  then [rumble] when flags bit1 is set: the profile's rumble
//                  options, else the default ones (what the controller uses
//                  today). A host fails a MAP whose bit1 block is short.
//   0x83 OK        [request_type u8]
//   0x84 ERROR     [request_type u8][code u32][utf8 message]
//                  code = the POSIX errno value of the std::errc the service
//                  chose (informational; the message is authoritative)
//   0x85 PROFILES  [count u8]{[vid u16][pid u16][name str][rule_count u8]}
//   0x86 INPUT     [flags u8][raw state][mapped state]
//                  flags bit0 = a controller is connected, bit1 = the state
//                  changed since the previous INPUT event (streamed only)
// Replies echo the request frame's correlation id (if it carried one);
// streamed INPUT events and unsolicited INFO carry none.
//
// Compatibility: the device rejects trailing bytes in a request (a request
// listed with no payload must have none), but a host
// decodes device -> host messages leniently: bytes after the fields it knows
// are ignored, so a later version of the protocol may append fields to them.
//
// Revision: rumble options (still v1). The rumble additions (INFO flags bit1,
// MAP flags bit1 + its block, SET_MAP's optional block) keep the protocol at
// version 1 and are found through capability bits instead, because a released
// console hides a module advertised at another version. Every addition is
// compatible both ways: an older host tests only bit0 and ignores the bytes
// after the map, and its SET_MAP (no block) keeps the stored rumble options; a
// newer host sees INFO bit1 clear on older firmware and sends SET_MAP without
// the block, byte-identical to before. Firmware that does not know the block
// rejects a SET_MAP that carries one as malformed (nothing is saved).
// TEST_RUMBLE (0x08) is offered only behind INFO flags bit2, so a host never
// sends it to firmware that does not know it.

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "gamepad_map.hpp"
#include "gamepad_state.hpp"
#include "rumble_options.hpp"
#include "stream_frame.hpp"

namespace espp::detail::gamepad_map_protocol {

/// Default dispatcher module id (a routing key only; see GamepadMapService::Config::module).
inline constexpr uint8_t kModule = 11;
/// Stable protocol identifier + version advertised through discovery.
inline constexpr const char *kProtocol = "espp.gamepad_map";
inline constexpr uint16_t kProtocolVersion = 1;

/// Frame `type` values within the gamepad map module.
enum class Type : uint8_t {
  // host -> device
  GetInfo = 0x01,
  GetMap = 0x02,
  SetMap = 0x03,
  EraseProfile = 0x04,
  ListProfiles = 0x05,
  SetStream = 0x06,
  ResetAll = 0x07,
  TestRumble = 0x08,
  // device -> host (high bit set)
  Info = 0x81,
  Map = 0x82,
  Ok = 0x83,
  Error = 0x84,
  Profiles = 0x85,
  Input = 0x86,
};

/// INFO / INPUT flag: a controller is connected.
inline constexpr uint8_t kFlagConnected = 0x01;
/// INPUT flag: the state changed since the previous INPUT event.
inline constexpr uint8_t kFlagChanged = 0x02;
/// MAP flag: the requested map exists (a profile that does not answers with the default map).
inline constexpr uint8_t kFlagExists = 0x01;
/// INFO flag: the device stores rumble options per map (MAP carries them, SET_MAP accepts them).
inline constexpr uint8_t kFlagRumbleOptions = 0x02;
/// INFO flag: the device takes TEST_RUMBLE (plays a test pattern through given options).
inline constexpr uint8_t kFlagTestRumble = 0x04;
/// MAP flag: a rumble block follows the map.
inline constexpr uint8_t kFlagRumble = 0x02;

/// Bytes of an encoded GamepadState.
inline constexpr size_t kStateSize = 16;
/// Bytes of one encoded GamepadMapRule.
inline constexpr size_t kRuleSize = 6;
/// Bytes of an encoded RumbleOptions.
inline constexpr size_t kRumbleSize = 2;

/// Which map a GET_MAP / SET_MAP addresses.
struct Target {
  bool profile{false}; ///< false = the default map
  uint16_t vid{0};
  uint16_t pid{0};
  std::string name{};
  bool operator==(const Target &) const = default;
};

/// The connected controller, as carried in INFO.
struct Controller {
  uint16_t vid{0};
  uint16_t pid{0};
  std::string name{};
  bool has_profile{false};
  bool operator==(const Controller &) const = default;
};

/// Decoded INFO.
struct Info {
  uint8_t button_count{0};
  uint8_t axis_count{0};
  uint8_t max_rules{0};
  uint8_t max_profiles{0};
  uint8_t profile_count{0};
  bool rumble_options{false}; ///< flags bit1: the device stores rumble options per map
  bool test_rumble{false};    ///< flags bit2: the device takes TEST_RUMBLE
  std::optional<Controller> controller{};
  bool operator==(const Info &) const = default;
};

/// One entry of PROFILES.
struct ProfileSummary {
  uint16_t vid{0};
  uint16_t pid{0};
  std::string name{};
  uint8_t rule_count{0};
  bool operator==(const ProfileSummary &) const = default;
};

/// Decoded SET_STREAM.
struct StreamRequest {
  bool enable{false};
  uint16_t period_ms{0};
};

/// Decoded INPUT.
struct Input {
  bool connected{false};
  bool changed{false};
  GamepadState raw{};
  GamepadState mapped{};
};

/// Whether a type value is a device->host reply / event.
inline constexpr bool is_reply(Type type) { return (static_cast<uint8_t>(type) & 0x80) != 0; }

/// Build an encoded frame for a gamepad map message (device->host types map to
/// the frame reply flag).
inline std::vector<uint8_t> build_frame(Type type, std::span<const uint8_t> payload = {},
                                        uint8_t module = kModule,
                                        std::optional<uint16_t> correlation = std::nullopt) {
  return espp::stream_frame::build_frame(is_reply(type), module, static_cast<uint8_t>(type),
                                         payload, correlation);
}

// ---- byte helpers -------------------------------------------------------------

/// Reads little-endian fields from a payload; every read fails (and keeps
/// failing) once the payload is exhausted.
class Reader {
public:
  explicit Reader(std::span<const uint8_t> data)
      : data_(data) {}
  bool u8(uint8_t &v) {
    if (!have(1))
      return false;
    v = data_[pos_++];
    return true;
  }
  bool u16(uint16_t &v) {
    if (!have(2))
      return false;
    v = static_cast<uint16_t>(data_[pos_] | (data_[pos_ + 1] << 8));
    pos_ += 2;
    return true;
  }
  bool i16(int16_t &v) {
    uint16_t u = 0;
    if (!u16(u))
      return false;
    v = static_cast<int16_t>(u);
    return true;
  }
  bool u32(uint32_t &v) {
    if (!have(4))
      return false;
    v = static_cast<uint32_t>(data_[pos_]) | (static_cast<uint32_t>(data_[pos_ + 1]) << 8) |
        (static_cast<uint32_t>(data_[pos_ + 2]) << 16) |
        (static_cast<uint32_t>(data_[pos_ + 3]) << 24);
    pos_ += 4;
    return true;
  }
  bool str(std::string &s) {
    uint8_t n = 0;
    if (!u8(n) || !have(n))
      return false;
    s.assign(reinterpret_cast<const char *>(data_.data() + pos_), n);
    pos_ += n;
    return true;
  }
  bool done() const { return ok_ && pos_ == data_.size(); }
  /// Bytes not read yet (0 once a read has failed).
  size_t remaining() const { return ok_ ? data_.size() - pos_ : 0; }
  /// For device -> host messages: every known field was read (trailing bytes,
  /// fields of a later protocol version, are ignored).
  bool known_fields_read() const { return ok_; }

private:
  bool have(size_t n) {
    if (!ok_ || data_.size() - pos_ < n) {
      ok_ = false;
      return false;
    }
    return true;
  }
  std::span<const uint8_t> data_;
  size_t pos_{0};
  bool ok_{true};
};

inline void put_u8(std::vector<uint8_t> &out, uint8_t v) { out.push_back(v); }
inline void put_u16(std::vector<uint8_t> &out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>(v >> 8));
}
inline void put_i16(std::vector<uint8_t> &out, int16_t v) {
  put_u16(out, static_cast<uint16_t>(v));
}
inline void put_u32(std::vector<uint8_t> &out, uint32_t v) {
  for (int i = 0; i < 4; i++)
    out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}
/// Append a [len u8][bytes] string (truncated to 255 bytes).
inline void put_str(std::vector<uint8_t> &out, std::string_view s) {
  const size_t n = s.size() > 255 ? 255 : s.size();
  out.push_back(static_cast<uint8_t>(n));
  out.insert(out.end(), s.begin(), s.begin() + static_cast<std::ptrdiff_t>(n));
}

// ---- shared encodings -----------------------------------------------------------

inline void put_target(std::vector<uint8_t> &out, const Target &t) {
  put_u8(out, t.profile ? 1 : 0);
  if (!t.profile)
    return;
  put_u16(out, t.vid);
  put_u16(out, t.pid);
  put_str(out, t.name);
}
inline bool read_target(Reader &r, Target &t) {
  uint8_t kind = 0;
  if (!r.u8(kind) || kind > 1)
    return false;
  t = {};
  if (kind == 0)
    return true;
  t.profile = true;
  return r.u16(t.vid) && r.u16(t.pid) && r.str(t.name);
}

/// Encode a map. At most 255 rules are written (a valid map has far fewer).
inline void put_map(std::vector<uint8_t> &out, const GamepadMap &m) {
  const size_t n = m.rules.size() > 255 ? 255 : m.rules.size();
  put_u8(out, m.left_deadzone_percent);
  put_u8(out, m.right_deadzone_percent);
  put_u8(out, static_cast<uint8_t>(n));
  for (size_t i = 0; i < n; i++) {
    const auto &rule = m.rules[i];
    put_u8(out, static_cast<uint8_t>(rule.kind));
    put_u8(out, rule.source);
    put_u8(out, rule.target);
    put_u8(out, rule.flags);
    put_i16(out, rule.param);
  }
}
/// Decode a map. Only the layout is checked here; check the result with
/// GamepadMap::valid() before using it.
inline bool read_map(Reader &r, GamepadMap &m) {
  uint8_t count = 0;
  m = {};
  if (!r.u8(m.left_deadzone_percent) || !r.u8(m.right_deadzone_percent) || !r.u8(count))
    return false;
  m.rules.reserve(count);
  for (uint8_t i = 0; i < count; i++) {
    uint8_t kind = 0;
    GamepadMapRule rule;
    if (!r.u8(kind) || !r.u8(rule.source) || !r.u8(rule.target) || !r.u8(rule.flags) ||
        !r.i16(rule.param))
      return false;
    rule.kind = static_cast<GamepadMapRule::Kind>(kind);
    m.rules.push_back(rule);
  }
  return true;
}

/// Encode rumble options: [flags u8][strength_percent u8].
inline void put_rumble(std::vector<uint8_t> &out, const RumbleOptions &o) {
  put_u8(out, o.flags);
  put_u8(out, o.strength_percent);
}
/// Decode rumble options. Only the layout is checked here; check the result
/// with RumbleOptions::valid() before using it.
inline bool read_rumble(Reader &r, RumbleOptions &o) {
  o = {};
  return r.u8(o.flags) && r.u8(o.strength_percent);
}

inline void put_state(std::vector<uint8_t> &out, const GamepadState &s) {
  put_u32(out, s.buttons);
  for (const int16_t a : s.axes)
    put_i16(out, a);
}
inline bool read_state(Reader &r, GamepadState &s) {
  if (!r.u32(s.buttons))
    return false;
  for (auto &a : s.axes)
    if (!r.i16(a))
      return false;
  return true;
}

// ---- requests (host -> device) --------------------------------------------------

inline std::vector<uint8_t> encode_get_map(const Target &t) {
  std::vector<uint8_t> p;
  put_target(p, t);
  return p;
}
inline std::optional<Target> decode_get_map(std::span<const uint8_t> payload) {
  Reader r(payload);
  Target t;
  if (!read_target(r, t) || !r.done())
    return std::nullopt;
  return t;
}

/// Decoded SET_MAP.
struct SetMapRequest {
  Target target{};
  GamepadMap map{};
  /// The rumble block, if the request carried one (absent: keep the stored options).
  std::optional<RumbleOptions> rumble{};
  bool operator==(const SetMapRequest &) const = default;
};

/// Encode SET_MAP; without `rumble` the request is the original v1 form (the
/// device keeps the stored rumble options).
inline std::vector<uint8_t>
encode_set_map(const Target &t, const GamepadMap &m,
               const std::optional<RumbleOptions> &rumble = std::nullopt) {
  std::vector<uint8_t> p;
  put_target(p, t);
  put_map(p, m);
  if (rumble)
    put_rumble(p, *rumble);
  return p;
}
/// Decode SET_MAP: exactly 0 (no rumble) or kRumbleSize (a rumble block) bytes
/// may follow the map. Only the layout is checked (see read_map / read_rumble).
inline std::optional<SetMapRequest> decode_set_map(std::span<const uint8_t> payload) {
  Reader r(payload);
  SetMapRequest req;
  if (!read_target(r, req.target) || !read_map(r, req.map))
    return std::nullopt;
  if (r.remaining() == kRumbleSize) {
    RumbleOptions o;
    if (!read_rumble(r, o))
      return std::nullopt;
    req.rumble = o;
  }
  if (!r.done())
    return std::nullopt;
  return req;
}

inline std::vector<uint8_t> encode_erase_profile(uint16_t vid, uint16_t pid,
                                                 std::string_view name) {
  std::vector<uint8_t> p;
  put_u16(p, vid);
  put_u16(p, pid);
  put_str(p, name);
  return p;
}
inline std::optional<Target> decode_erase_profile(std::span<const uint8_t> payload) {
  Reader r(payload);
  Target t;
  t.profile = true;
  if (!r.u16(t.vid) || !r.u16(t.pid) || !r.str(t.name) || !r.done())
    return std::nullopt;
  return t;
}

inline std::vector<uint8_t> encode_set_stream(bool enable, uint16_t period_ms) {
  std::vector<uint8_t> p;
  put_u8(p, enable ? 1 : 0);
  put_u16(p, period_ms);
  return p;
}
inline std::optional<StreamRequest> decode_set_stream(std::span<const uint8_t> payload) {
  Reader r(payload);
  uint8_t enable = 0;
  StreamRequest s;
  if (!r.u8(enable) || enable > 1 || !r.u16(s.period_ms) || !r.done())
    return std::nullopt;
  s.enable = enable != 0;
  return s;
}

/// Encode TEST_RUMBLE: [rumble].
inline std::vector<uint8_t> encode_test_rumble(const RumbleOptions &rumble) {
  std::vector<uint8_t> p;
  put_rumble(p, rumble);
  return p;
}
/// Decode TEST_RUMBLE: exactly one rumble block (layout only; check valid()).
inline std::optional<RumbleOptions> decode_test_rumble(std::span<const uint8_t> payload) {
  Reader r(payload);
  RumbleOptions o;
  if (!read_rumble(r, o) || !r.done())
    return std::nullopt;
  return o;
}

// ---- replies / events (device -> host) -----------------------------------------

inline std::vector<uint8_t> encode_info(const Info &info) {
  std::vector<uint8_t> p;
  put_u8(p, info.button_count);
  put_u8(p, info.axis_count);
  put_u8(p, info.max_rules);
  put_u8(p, info.max_profiles);
  put_u8(p, info.profile_count);
  put_u8(p, static_cast<uint8_t>((info.controller ? kFlagConnected : 0) |
                                 (info.rumble_options ? kFlagRumbleOptions : 0) |
                                 (info.test_rumble ? kFlagTestRumble : 0)));
  if (info.controller) {
    put_u16(p, info.controller->vid);
    put_u16(p, info.controller->pid);
    put_str(p, info.controller->name);
    put_u8(p, info.controller->has_profile ? 1 : 0);
  }
  return p;
}
inline std::optional<Info> decode_info(std::span<const uint8_t> payload) {
  Reader r(payload);
  Info info;
  uint8_t flags = 0;
  if (!r.u8(info.button_count) || !r.u8(info.axis_count) || !r.u8(info.max_rules) ||
      !r.u8(info.max_profiles) || !r.u8(info.profile_count) || !r.u8(flags))
    return std::nullopt;
  info.rumble_options = flags & kFlagRumbleOptions;
  info.test_rumble = flags & kFlagTestRumble;
  if (flags & kFlagConnected) {
    Controller c;
    uint8_t has = 0;
    if (!r.u16(c.vid) || !r.u16(c.pid) || !r.str(c.name) || !r.u8(has))
      return std::nullopt;
    c.has_profile = has != 0;
    info.controller = std::move(c);
  }
  if (!r.known_fields_read())
    return std::nullopt;
  return info;
}

/// Decoded MAP.
struct MapReply {
  Target target{};
  bool exists{false};
  GamepadMap map{};
  /// The rumble block (flags bit1), if the device sent one.
  std::optional<RumbleOptions> rumble{};
};

/// Encode MAP; with `rumble`, flags bit1 is set and the block follows the map.
inline std::vector<uint8_t>
encode_map_reply(const Target &t, bool exists, const GamepadMap &m,
                 const std::optional<RumbleOptions> &rumble = std::nullopt) {
  std::vector<uint8_t> p;
  put_target(p, t);
  put_u8(p, static_cast<uint8_t>((exists ? kFlagExists : 0) | (rumble ? kFlagRumble : 0)));
  put_map(p, m);
  if (rumble)
    put_rumble(p, *rumble);
  return p;
}
/// Decode MAP. Flags bit1 with a missing or short rumble block fails; bytes
/// after the known fields are ignored (without bit1 there is no rumble block,
/// whatever follows the map).
inline std::optional<MapReply> decode_map_reply(std::span<const uint8_t> payload) {
  Reader r(payload);
  MapReply reply;
  uint8_t flags = 0;
  if (!read_target(r, reply.target) || !r.u8(flags) || !read_map(r, reply.map))
    return std::nullopt;
  reply.exists = flags & kFlagExists;
  if (flags & kFlagRumble) {
    RumbleOptions o;
    if (!read_rumble(r, o))
      return std::nullopt;
    reply.rumble = o;
  }
  if (!r.known_fields_read())
    return std::nullopt;
  return reply;
}

inline std::vector<uint8_t> encode_ok(uint8_t request_type) { return {request_type}; }

inline std::vector<uint8_t> encode_error(uint8_t request_type, uint32_t code,
                                         std::string_view message) {
  std::vector<uint8_t> p;
  put_u8(p, request_type);
  put_u32(p, code);
  p.insert(p.end(), message.begin(), message.end());
  return p;
}

/// Encode PROFILES, keeping it within @p max_bytes: entries that would not fit
/// are dropped from the end (at most 255 entries).
/// @param[out] encoded_count Set to the number of entries encoded, if non-null.
inline std::vector<uint8_t> encode_profiles(std::span<const ProfileSummary> profiles,
                                            size_t max_bytes = espp::stream_frame::kMaxPayloadSize,
                                            size_t *encoded_count = nullptr) {
  std::vector<uint8_t> p{0};
  size_t n = 0;
  for (const auto &s : profiles) {
    const size_t name_len = s.name.size() > 255 ? 255 : s.name.size();
    const size_t entry = 2 + 2 + 1 + name_len + 1;
    if (n == 255 || p.size() + entry > max_bytes)
      break;
    put_u16(p, s.vid);
    put_u16(p, s.pid);
    put_str(p, s.name);
    put_u8(p, s.rule_count);
    n++;
  }
  p[0] = static_cast<uint8_t>(n);
  if (encoded_count)
    *encoded_count = n;
  return p;
}
inline std::optional<std::vector<ProfileSummary>>
decode_profiles(std::span<const uint8_t> payload) {
  Reader r(payload);
  uint8_t count = 0;
  if (!r.u8(count))
    return std::nullopt;
  std::vector<ProfileSummary> out(count);
  for (auto &s : out)
    if (!r.u16(s.vid) || !r.u16(s.pid) || !r.str(s.name) || !r.u8(s.rule_count))
      return std::nullopt;
  if (!r.known_fields_read())
    return std::nullopt;
  return out;
}

inline std::vector<uint8_t> encode_input(const Input &in) {
  std::vector<uint8_t> p;
  p.reserve(1 + 2 * kStateSize);
  put_u8(p, static_cast<uint8_t>((in.connected ? kFlagConnected : 0) |
                                 (in.changed ? kFlagChanged : 0)));
  put_state(p, in.raw);
  put_state(p, in.mapped);
  return p;
}
inline std::optional<Input> decode_input(std::span<const uint8_t> payload) {
  Reader r(payload);
  Input in;
  uint8_t flags = 0;
  if (!r.u8(flags) || !read_state(r, in.raw) || !read_state(r, in.mapped) || !r.known_fields_read())
    return std::nullopt;
  in.connected = flags & kFlagConnected;
  in.changed = flags & kFlagChanged;
  return in;
}

} // namespace espp::detail::gamepad_map_protocol
