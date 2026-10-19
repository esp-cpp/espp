#pragma once

// Wire protocol of the VL53L5CX console: a small command set that lets a host
// (the hosted vl53l5cx_console.html web app, or any other client) configure an
// espp::Vl53l5cx multizone ToF sensor that runs ON the device, start / stop
// ranging, and receive every ranging frame (4x4 / 8x8 zones) as it is read.
//
// Framed with the espp stream_frame v2 codec and routed by an espp::Dispatcher
// on MODULE ID 10 (kModuleId). Host->device requests use type 0x1X; the
// device->host replies / events use 0x9X (their high bit is the frame reply
// flag). All multi-byte fields are little-endian.
//
// This header is standard-library only (host-testable, see
// test/vl53l5cx_protocol_host_test.cpp); espp::Vl53l5cxService
// (vl53l5cx_service.hpp) implements the device side.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace espp {
namespace vl53l5cx_protocol {

/// Default dispatcher module id of the VL53L5CX console protocol (a routing
/// key; Vl53l5cxService::Config::module can move it).
static constexpr uint8_t kModuleId = 10;
/// Stable protocol identifier + version advertised through discovery
/// (Dispatcher::ModuleInfo::protocol); the console locates the module by this
/// rather than by its module id.
static constexpr const char *kProtocol = "espp.vl53l5cx";
static constexpr uint16_t kProtocolVersion = 1;

/// Host -> device requests (0x1X).
enum class Request : uint8_t {
  GetStatus = 0x10,             ///< request one STATUS snapshot (no payload)
  SetResolution = 0x11,         ///< [zones u8] 16 (4x4) or 64 (8x8); not while ranging
  SetRangingFrequency = 0x12,   ///< [hz u8] 1..60 (4x4) / 1..15 (8x8); not while ranging
  SetIntegrationTime = 0x13,    ///< [ms u32] 2..1000 (autonomous mode); not while ranging
  SetSharpener = 0x14,          ///< [percent u8] 0..99; not while ranging
  SetTargetOrder = 0x15,        ///< [order u8] 1 closest / 2 strongest; not while ranging
  SetRangingMode = 0x16,        ///< [mode u8] 1 continuous / 3 autonomous; not while ranging
  SetPowerMode = 0x17,          ///< [mode u8] 0 sleep / 1 wakeup; not while ranging
  EnableMotionIndicator = 0x18, ///< [min_mm u16][max_mm u16]; not while ranging. Only an
                                ///< INITIALIZE disables it again (the sensor has no "off").
  StartRanging = 0x19,          ///< start ranging with the current settings (no payload)
  StopRanging = 0x1A,           ///< stop ranging (no payload)
  Initialize = 0x1B,            ///< reboot the sensor + re-upload its firmware (~2 s; the
                                ///< reply comes after); resets every setting (no payload)
  SetFrameStream = 0x1C,        ///< [enable u8][divider u8] send every divider-th frame
                                ///< read on this transport (0 = device default)
  GetFrame = 0x1D,              ///< request the latest FRAME (ERROR when none was read yet)
  SetI2cAddress = 0x1E,         ///< [address u8] 7-bit, volatile (back to 0x29 at reset)
};

/// Device -> host replies / events (0x9X => reply flag set).
enum class Reply : uint8_t {
  Status = 0x90, ///< status snapshot (see Status)
  Frame = 0x91,  ///< one ranging frame (see Frame)
  Ok = 0x92,     ///< ack for a request: [request_type u8]
  Error = 0x93,  ///< failure: [request_type u8][code u32][utf8 message]
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
inline void put_i16(std::vector<uint8_t> &out, int16_t v) {
  put_u16(out, static_cast<uint16_t>(v));
}
inline uint16_t get_u16(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[off]) |
                               (static_cast<uint16_t>(p[off + 1]) << 8));
}
inline uint32_t get_u32(std::span<const uint8_t> p, size_t off) {
  return static_cast<uint32_t>(p[off]) | (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) | (static_cast<uint32_t>(p[off + 3]) << 24);
}
inline int16_t get_i16(std::span<const uint8_t> p, size_t off) {
  return static_cast<int16_t>(get_u16(p, off));
}

// ---- request payloads -------------------------------------------------------------

/// A one-byte request payload (SetResolution, SetRangingFrequency, SetSharpener,
/// SetTargetOrder, SetRangingMode, SetPowerMode, SetI2cAddress).
struct ByteArg {
  uint8_t value{0};
  static constexpr size_t kSize = 1;
  static std::optional<ByteArg> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return ByteArg{p[0]};
  }
  std::vector<uint8_t> serialize() const { return {value}; }
};

/// SetIntegrationTime payload: [ms u32] (4 B).
struct SetIntegrationTime {
  uint32_t time_ms{0};
  static constexpr size_t kSize = 4;
  static std::optional<SetIntegrationTime> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetIntegrationTime{get_u32(p, 0)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_u32(out, time_ms);
    return out;
  }
};

/// EnableMotionIndicator payload: [min_mm u16][max_mm u16] (4 B).
struct EnableMotionIndicator {
  uint16_t min_mm{400};
  uint16_t max_mm{1500};
  static constexpr size_t kSize = 4;
  static std::optional<EnableMotionIndicator> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return EnableMotionIndicator{get_u16(p, 0), get_u16(p, 2)};
  }
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    put_u16(out, min_mm);
    put_u16(out, max_mm);
    return out;
  }
};

/// SetFrameStream payload: [enable u8][divider u8] (2 B); divider 0 = default.
struct SetFrameStream {
  bool enabled{false};
  uint8_t divider{0};
  static constexpr size_t kSize = 2;
  static std::optional<SetFrameStream> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    return SetFrameStream{p[0] != 0, p[1]};
  }
  std::vector<uint8_t> serialize() const {
    return {static_cast<uint8_t>(enabled ? 1 : 0), divider};
  }
};

// ---- reply payloads ----------------------------------------------------------------

/// STATUS payload (24 B): [flags u8][variant u8][address u8][zones u8]
/// [frequency_hz u8][integration_ms u32][sharpener u8][target_order u8]
/// [ranging_mode u8][power_mode u8][motion_min_mm u16][motion_max_mm u16]
/// [frame_count u32][frame_rate_chz u16][stream_divider u8].
/// The sensor settings are the values read back from the sensor (its own
/// defaults after an INITIALIZE); while ranging they are the last values read
/// or set (the firmware cannot be queried mid-ranging).
struct Status {
  bool initialized{false};     ///< the firmware is uploaded and the sensor answers
  bool ranging{false};         ///< between a successful START and STOP
  bool motion_enabled{false};  ///< the motion indicator is configured
  bool stream_enabled{false};  ///< frames are streamed on this transport
  uint8_t variant{0};          ///< 0 VL53L5CX / 1 VL53L7CX (the firmware image uploaded)
  uint8_t address{0x29};       ///< current 7-bit I2C address
  uint8_t zones{16};           ///< 16 (4x4) or 64 (8x8)
  uint8_t frequency_hz{1};     ///< ranging frequency
  uint32_t integration_ms{10}; ///< integration time (autonomous mode)
  uint8_t sharpener{5};        ///< sharpener, percent
  uint8_t target_order{2};     ///< 1 closest / 2 strongest
  uint8_t ranging_mode{3};     ///< 1 continuous / 3 autonomous
  uint8_t power_mode{1};       ///< 0 sleep / 1 wakeup
  uint16_t motion_min_mm{0};   ///< motion window (0 when the indicator is off)
  uint16_t motion_max_mm{0};
  uint32_t frame_count{0};    ///< frames read since boot
  uint16_t frame_rate_chz{0}; ///< measured frame rate, centi-Hz (0 while idle)
  uint8_t stream_divider{1};  ///< every divider-th frame is streamed

  static constexpr uint8_t kFlagInitialized = 0x01;
  static constexpr uint8_t kFlagRanging = 0x02;
  static constexpr uint8_t kFlagMotionEnabled = 0x04;
  static constexpr uint8_t kFlagStreamEnabled = 0x08;
  static constexpr size_t kSize = 24;

  bool operator==(const Status &) const = default;

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    out.reserve(kSize);
    uint8_t flags = 0;
    if (initialized)
      flags |= kFlagInitialized;
    if (ranging)
      flags |= kFlagRanging;
    if (motion_enabled)
      flags |= kFlagMotionEnabled;
    if (stream_enabled)
      flags |= kFlagStreamEnabled;
    out.push_back(flags);
    out.push_back(variant);
    out.push_back(address);
    out.push_back(zones);
    out.push_back(frequency_hz);
    put_u32(out, integration_ms);
    out.push_back(sharpener);
    out.push_back(target_order);
    out.push_back(ranging_mode);
    out.push_back(power_mode);
    put_u16(out, motion_min_mm);
    put_u16(out, motion_max_mm);
    put_u32(out, frame_count);
    put_u16(out, frame_rate_chz);
    out.push_back(stream_divider);
    return out;
  }

  static std::optional<Status> parse(std::span<const uint8_t> p) {
    if (p.size() < kSize)
      return std::nullopt;
    Status s;
    const uint8_t flags = p[0];
    s.initialized = (flags & kFlagInitialized) != 0;
    s.ranging = (flags & kFlagRanging) != 0;
    s.motion_enabled = (flags & kFlagMotionEnabled) != 0;
    s.stream_enabled = (flags & kFlagStreamEnabled) != 0;
    s.variant = p[1];
    s.address = p[2];
    s.zones = p[3];
    s.frequency_hz = p[4];
    s.integration_ms = get_u32(p, 5);
    s.sharpener = p[9];
    s.target_order = p[10];
    s.ranging_mode = p[11];
    s.power_mode = p[12];
    s.motion_min_mm = get_u16(p, 13);
    s.motion_max_mm = get_u16(p, 15);
    s.frame_count = get_u32(p, 17);
    s.frame_rate_chz = get_u16(p, 21);
    s.stream_divider = p[23];
    return s;
  }
};

/// One zone of a FRAME (19 B on the wire): [distance_mm i16][status u8]
/// [nb_targets u8][reflectance_pct u8][sigma_mm u16][signal_kcps u32]
/// [ambient_kcps u32][spads u32].
struct Zone {
  int16_t distance_mm{0};
  uint8_t status{0}; ///< target status (5 / 9 = valid, 255 = no target)
  uint8_t nb_targets{0};
  uint8_t reflectance_pct{0};
  uint16_t sigma_mm{0};
  uint32_t signal_kcps{0}; ///< return signal, kcps / SPAD
  uint32_t ambient_kcps{0};
  uint32_t spads{0};
  static constexpr size_t kSize = 19;
  bool operator==(const Zone &) const = default;
};

/// The motion indicator block of a FRAME (140 B on the wire, present when the
/// frame's kFlagMotion is set): [global_1 u32][global_2 u32][status u8]
/// [detected u8][aggregates u8][spare u8][motion u32 x 32].
struct Motion {
  uint32_t global_indicator_1{0};
  uint32_t global_indicator_2{0};
  uint8_t status{0};
  uint8_t nb_of_detected_aggregates{0};
  uint8_t nb_of_aggregates{0};
  uint8_t spare{0};
  std::array<uint32_t, 32> motion{};
  static constexpr size_t kSize = 4 + 4 + 4 + 32 * 4; // = 140
  bool operator==(const Motion &) const = default;
};

/// FRAME payload: [sequence u32][timestamp_ms u32][zones u8][flags u8], then
/// `zones` Zone records (row-major, zone = row * width + col), then a Motion
/// block when flags has kFlagMotion. 8x8 with motion = 1366 B.
struct Frame {
  uint32_t sequence{0};     ///< frames read since boot (this one's number)
  uint32_t timestamp_ms{0}; ///< device uptime when the frame was read
  uint8_t zones{16};        ///< 16 or 64
  bool motion_present{false};
  std::array<Zone, 64> zone{};
  Motion motion{};

  static constexpr uint8_t kFlagMotion = 0x01;
  static constexpr size_t kHeaderSize = 10;
  static constexpr size_t kMaxSize = kHeaderSize + 64 * Zone::kSize + Motion::kSize;

  /// Zones per row (4 or 8)
  size_t width() const { return zones == 64 ? 8 : 4; }

  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> out;
    const size_t n = zones == 64 ? 64 : 16;
    out.reserve(kHeaderSize + n * Zone::kSize + (motion_present ? Motion::kSize : 0));
    put_u32(out, sequence);
    put_u32(out, timestamp_ms);
    out.push_back(static_cast<uint8_t>(n));
    out.push_back(motion_present ? kFlagMotion : 0);
    for (size_t i = 0; i < n; i++) {
      const Zone &z = zone[i];
      put_i16(out, z.distance_mm);
      out.push_back(z.status);
      out.push_back(z.nb_targets);
      out.push_back(z.reflectance_pct);
      put_u16(out, z.sigma_mm);
      put_u32(out, z.signal_kcps);
      put_u32(out, z.ambient_kcps);
      put_u32(out, z.spads);
    }
    if (motion_present) {
      put_u32(out, motion.global_indicator_1);
      put_u32(out, motion.global_indicator_2);
      out.push_back(motion.status);
      out.push_back(motion.nb_of_detected_aggregates);
      out.push_back(motion.nb_of_aggregates);
      out.push_back(motion.spare);
      for (uint32_t m : motion.motion)
        put_u32(out, m);
    }
    return out;
  }

  static std::optional<Frame> parse(std::span<const uint8_t> p) {
    if (p.size() < kHeaderSize)
      return std::nullopt;
    Frame f;
    f.sequence = get_u32(p, 0);
    f.timestamp_ms = get_u32(p, 4);
    f.zones = p[8];
    if (f.zones != 16 && f.zones != 64)
      return std::nullopt;
    f.motion_present = (p[9] & kFlagMotion) != 0;
    size_t off = kHeaderSize;
    if (p.size() < off + f.zones * Zone::kSize + (f.motion_present ? Motion::kSize : 0))
      return std::nullopt;
    for (size_t i = 0; i < f.zones; i++, off += Zone::kSize) {
      Zone &z = f.zone[i];
      z.distance_mm = get_i16(p, off);
      z.status = p[off + 2];
      z.nb_targets = p[off + 3];
      z.reflectance_pct = p[off + 4];
      z.sigma_mm = get_u16(p, off + 5);
      z.signal_kcps = get_u32(p, off + 7);
      z.ambient_kcps = get_u32(p, off + 11);
      z.spads = get_u32(p, off + 15);
    }
    if (f.motion_present) {
      f.motion.global_indicator_1 = get_u32(p, off);
      f.motion.global_indicator_2 = get_u32(p, off + 4);
      f.motion.status = p[off + 8];
      f.motion.nb_of_detected_aggregates = p[off + 9];
      f.motion.nb_of_aggregates = p[off + 10];
      f.motion.spare = p[off + 11];
      off += 12;
      for (auto &m : f.motion.motion) {
        m = get_u32(p, off);
        off += 4;
      }
    }
    return f;
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

} // namespace vl53l5cx_protocol
} // namespace espp
