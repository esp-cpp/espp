#pragma once

// Wire protocol of the MCP266 console: a small, HIGH-LEVEL command set that
// lets a host (the hosted mcp266_console.html web app, or any other client)
// drive an espp::Mcp266 that runs ON the device -- "configure axis", "move to
// position", "get status" -- without any CANopen/DS402 knowledge on the host.
//
// Framed with the espp stream_frame v2 codec and routed by an espp::Dispatcher
// on MODULE ID 6 (kModuleId). The `type` byte's high nibble is 6 for
// host->device requests and E for device->host replies/events; the reply types
// (0xE_) additionally set the frame reply flag (their high bit). All multi-byte
// fields are little-endian. The axis selector byte is 0 = M1, 1 = M2.
//
// This header is standard-library only (host-testable, see
// test/mcp266_protocol_host_test.cpp); espp::Mcp266Service (mcp266_service.hpp)
// implements the device side.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace espp {
namespace mcp266_protocol {

/// Dispatcher module id owned by the MCP266 console protocol.
static constexpr uint8_t kModuleId = 6;

/// Axis selector used in request payloads (matches espp::Mcp266::Axis order).
enum class Axis : uint8_t {
  M1 = 0,
  M2 = 1,
};

/// Host -> device requests (high nibble 6).
enum class Request : uint8_t {
  Start = 0x60,                 ///< NMT-start the node + clear latched faults (no payload)
  ResetFaults = 0x61,           ///< clear latched CiA 402 faults on both axes (no payload)
  ResetEstop = 0x62,            ///< attempt an e-stop reset (no payload)
  ConfigurePositionLoop = 0x63, ///< [axis u8][min i32][max i32][fallback_p i32]
  SetPositionLimits = 0x64,     ///< CiA 402 software limits: [axis u8][min i32][max i32]
  MoveToPosition = 0x65,        ///< [axis u8][target i32][vel u32][accel u32][decel u32]
  DriveSpeed = 0x66,            ///< [axis u8][qpps i32] (inert on tested firmware)
  DriveDuty = 0x67,             ///< [axis u8][duty i16] (inert on tested firmware)
  GetStatus = 0x68,             ///< request one STATUS snapshot (no payload)
  SetStatusStream = 0x69,       ///< [enable u8][period_ms u16] periodic STATUS streaming
                                ///< (period 0 = device default; clamped by the device)
  GetDeviceInfo = 0x6A,         ///< request DEVICE_INFO (no payload)
};

/// Device -> host replies / events (high nibble E => reply flag set).
enum class Reply : uint8_t {
  Status = 0xE0,     ///< status snapshot (see Status)
  Ok = 0xE1,         ///< ack for a request: [request_type u8]
  Error = 0xE2,      ///< failure: [request_type u8][code u32][utf8 message]
  DeviceInfo = 0xE3, ///< [device_type u32][utf8 name]
};

/// Whether a type byte is a device->host reply/event (the frame reply flag).
constexpr bool is_reply(uint8_t type) { return (type & 0x80) != 0; }

// ---- little-endian helpers -----------------------------------------------------

inline void put_u16(std::vector<uint8_t> &out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
}
inline void put_u32(std::vector<uint8_t> &out, uint32_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
  out.push_back(static_cast<uint8_t>(v >> 16));
  out.push_back(static_cast<uint8_t>(v >> 24));
}
inline void put_i32(std::vector<uint8_t> &out, int32_t v) {
  put_u32(out, static_cast<uint32_t>(v));
}
inline uint16_t get_u16(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[off]) |
                               (static_cast<uint16_t>(p[off + 1]) << 8));
}
inline uint32_t get_u32(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24);
}
inline int32_t get_i32(std::span<const uint8_t> p, size_t off) {
  return static_cast<int32_t>(get_u32(p, off));
}
inline int16_t get_i16(std::span<const uint8_t> p, size_t off) {
  return static_cast<int16_t>(get_u16(p, off));
}

/// Decode an axis selector byte; nullopt if it is not 0 or 1 (so an unexpected
/// value can never silently command the wrong motor).
constexpr std::optional<Axis> parse_axis(uint8_t b) {
  if (b > static_cast<uint8_t>(Axis::M2))
    return std::nullopt;
  return static_cast<Axis>(b);
}

// ---- request payloads -------------------------------------------------------------

/// ConfigurePositionLoop payload: [axis u8][min i32][max i32][fallback_p i32] (13 B).
struct ConfigurePositionLoop {
  Axis axis{Axis::M1};
  int32_t min{0};
  int32_t max{0};
  int32_t fallback_p{0};
  static constexpr size_t kSize = 13;
  static std::optional<ConfigurePositionLoop> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    const auto axis = parse_axis(p[0]);
    if (!axis)
      return std::nullopt;
    return ConfigurePositionLoop{*axis, get_i32(p, 1), get_i32(p, 5), get_i32(p, 9)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(axis)};
    put_i32(out, min);
    put_i32(out, max);
    put_i32(out, fallback_p);
    return out;
  }
};

/// SetPositionLimits payload: [axis u8][min i32][max i32] (9 B).
struct SetPositionLimits {
  Axis axis{Axis::M1};
  int32_t min{0};
  int32_t max{0};
  static constexpr size_t kSize = 9;
  static std::optional<SetPositionLimits> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    const auto axis = parse_axis(p[0]);
    if (!axis)
      return std::nullopt;
    return SetPositionLimits{*axis, get_i32(p, 1), get_i32(p, 5)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(axis)};
    put_i32(out, min);
    put_i32(out, max);
    return out;
  }
};

/// MoveToPosition payload: [axis u8][target i32][vel u32][accel u32][decel u32] (17 B).
struct MoveToPosition {
  Axis axis{Axis::M1};
  int32_t target{0};
  uint32_t velocity{0};
  uint32_t accel{0};
  uint32_t decel{0};
  static constexpr size_t kSize = 17;
  static std::optional<MoveToPosition> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    const auto axis = parse_axis(p[0]);
    if (!axis)
      return std::nullopt;
    return MoveToPosition{*axis, get_i32(p, 1), get_u32(p, 5), get_u32(p, 9), get_u32(p, 13)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(axis)};
    put_i32(out, target);
    put_u32(out, velocity);
    put_u32(out, accel);
    put_u32(out, decel);
    return out;
  }
};

/// DriveSpeed payload: [axis u8][qpps i32] (5 B).
struct DriveSpeed {
  Axis axis{Axis::M1};
  int32_t qpps{0};
  static constexpr size_t kSize = 5;
  static std::optional<DriveSpeed> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    const auto axis = parse_axis(p[0]);
    if (!axis)
      return std::nullopt;
    return DriveSpeed{*axis, get_i32(p, 1)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(axis)};
    put_i32(out, qpps);
    return out;
  }
};

/// DriveDuty payload: [axis u8][duty i16] (3 B).
struct DriveDuty {
  Axis axis{Axis::M1};
  int16_t duty{0};
  static constexpr size_t kSize = 3;
  static std::optional<DriveDuty> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    const auto axis = parse_axis(p[0]);
    if (!axis)
      return std::nullopt;
    return DriveDuty{*axis, get_i16(p, 1)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(axis)};
    put_u16(out, static_cast<uint16_t>(duty));
    return out;
  }
};

/// SetStatusStream payload: [enable u8][period_ms u16] (3 B); period 0 = default.
struct SetStatusStream {
  bool enabled{false};
  uint16_t period_ms{0};
  static constexpr size_t kSize = 3;
  static std::optional<SetStatusStream> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetStatusStream{p[0] != 0, get_u16(p, 1)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out{static_cast<uint8_t>(enabled ? 1 : 0)};
    put_u16(out, period_ms);
    return out;
  }
};

// ---- reply payloads ----------------------------------------------------------------

/// One axis of a STATUS snapshot.
struct AxisStatus {
  int32_t position{0};    ///< encoder count
  int32_t velocity{0};    ///< counts per second
  uint16_t statusword{0}; ///< CiA 402 statusword
  bool operator==(const AxisStatus &) const = default;
};

/// STATUS payload (25 B): M1 then M2 as [position i32][velocity i32][statusword u16],
/// then [battery_decivolts u16][temp_decidegrees u16][flags u8]
/// (flags bit0 = the node responded to the last poll).
struct Status {
  AxisStatus m1;
  AxisStatus m2;
  uint16_t battery_decivolts{0}; ///< main battery, tenths of a volt
  uint16_t temp_decidegrees{0};  ///< controller temperature, tenths of a degree C
  bool online{false};            ///< the node responded to the last poll

  static constexpr uint8_t kFlagOnline = 0x01;
  static constexpr size_t kAxisSize = 10;
  static constexpr size_t kSize = 2 * kAxisSize + 2 + 2 + 1; // = 25

  bool operator==(const Status &) const = default;

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kSize);
    for (const AxisStatus *a : {&m1, &m2}) {
      put_i32(out, a->position);
      put_i32(out, a->velocity);
      put_u16(out, a->statusword);
    }
    put_u16(out, battery_decivolts);
    put_u16(out, temp_decidegrees);
    out.push_back(online ? kFlagOnline : 0);
    return out;
  }

  static std::optional<Status> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    Status s;
    size_t off = 0;
    for (AxisStatus *a : {&s.m1, &s.m2}) {
      a->position = get_i32(p, off);
      a->velocity = get_i32(p, off + 4);
      a->statusword = get_u16(p, off + 8);
      off += kAxisSize;
    }
    s.battery_decivolts = get_u16(p, off);
    s.temp_decidegrees = get_u16(p, off + 2);
    s.online = (p[off + 4] & kFlagOnline) != 0;
    return s;
  }
};

/// DEVICE_INFO payload: [device_type u32][utf8 name].
struct DeviceInfo {
  uint32_t device_type{0};
  std::string name;
  bool operator==(const DeviceInfo &) const = default;
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_u32(out, device_type);
    out.insert(out.end(), name.begin(), name.end());
    return out;
  }
  static std::optional<DeviceInfo> parse(std::span<const uint8_t> p) {
    if (p.size() < 4)
      return std::nullopt;
    return DeviceInfo{get_u32(p, 0), std::string(p.begin() + 4, p.end())};
  }
};

/// OK payload: [request_type u8].
inline std::vector<uint8_t> make_ok_payload(uint8_t request_type) { return {request_type}; }

/// ERROR payload: [request_type u8][code u32][utf8 message].
inline std::vector<uint8_t> make_error_payload(uint8_t request_type, uint32_t code,
                                               std::string_view message) {
  std::vector<uint8_t> out{request_type};
  put_u32(out, code);
  out.insert(out.end(), message.begin(), message.end());
  return out;
}

/// Decoded ERROR payload.
struct ErrorPayload {
  uint8_t request_type{0};
  uint32_t code{0};
  std::string message;
};

inline std::optional<ErrorPayload> parse_error_payload(std::span<const uint8_t> p) {
  if (p.size() < 5)
    return std::nullopt;
  return ErrorPayload{p[0], get_u32(p, 1), std::string(p.begin() + 5, p.end())};
}

} // namespace mcp266_protocol
} // namespace espp
