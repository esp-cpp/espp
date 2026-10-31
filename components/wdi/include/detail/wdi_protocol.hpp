#pragma once

// Wheelchair Digital Interface (WDI) — host-testable, ESP-free protocol core.
//
// Implements the Open-Mobility-Hub "Wheelchair HID" specification (v3.2): the
// report definitions, bitfields, the shared HID report descriptor, and the
// pack / parse helpers used by both the USB and BLE transports and by both the
// WDI device (the app / controller) and WDI host (the wheelchair) roles.
//
// Spec: https://open-mobility-hub.github.io/wheelchair-digital-interface/
//       docs/wheelchair/wheelchair-hid.html
//
// This header depends only on a C++20 standard library so it can be unit-tested
// on a host (see test/wdi_protocol_host_test.cpp). All multi-byte report fields
// are little-endian EXCEPT the 128-bit Host UUID, which is big-endian (network
// byte order) per the spec.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace espp {
namespace wdi {

/// @brief HID report IDs. Direction is from the WDI **device** (the app /
///        controller, which is the USB device / BLE peripheral) point of view:
///        an Input report is device→host, an Output report is host→device.
enum class ReportId : uint8_t {
  Control = 0x01,           ///< Input  (app→host), 18-byte payload: joystick + flags
  Feedback = 0x02,          ///< Output (host→app), 19-byte payload: status + telemetry
  RequestFeedback = 0x03,   ///< Input  (app→host), 1 byte: poll for a Feedback report
  Keepalive = 0x04,         ///< Input  (app→host), 1 byte: connection heartbeat
  KeepaliveResponse = 0x05, ///< Output (host→app), 16 byte: the host's 128-bit UUID
};

/// @brief On-the-wire payload sizes (excluding the leading HID report-id byte).
inline constexpr size_t kControlSize = 18;
inline constexpr size_t kFeedbackSize = 19;
inline constexpr size_t kRequestFeedbackSize = 1;
inline constexpr size_t kKeepaliveSize = 1;
inline constexpr size_t kKeepaliveResponseSize = 16;

/// @brief The single-byte value carried by the Request Feedback (0x03) and
///        Keepalive (0x04) reports.
inline constexpr uint8_t kTriggerValue = 0x01;

/// @brief Bits of the Control report's "Standard1" u32 bitfield (bytes 2..5).
///        `Modifier` reverses the direction of the seating actuators in the high
///        byte (e.g. Tilt|Modifier = tilt backward). A "release" is all-zero.
enum class ControlBit : uint32_t {
  Modifier = 1u << 0,
  Stop = 1u << 1,
  DriveEnable = 1u << 2,
  CycleProfile = 1u << 3,
  Hazards = 1u << 4,
  CycleMode = 1u << 5,
  SpeedDown = 1u << 6,
  SpeedUp = 1u << 7,
  LeftBlinker = 1u << 8,
  RightBlinker = 1u << 9,
  Menu = 1u << 10,
  ProfileUp = 1u << 11,
  DriveDisable = 1u << 12,
  Headlights = 1u << 13,
  Horn = 1u << 14,
  ProfileDown = 1u << 15,
  Memory1 = 1u << 16,
  Memory2 = 1u << 17,
  Memory3 = 1u << 18,
  Memory4 = 1u << 19,
  Memory5 = 1u << 20,
  Memory6 = 1u << 21,
  MemoryHome = 1u << 22,
  // bit 23 reserved
  Tilt = 1u << 24,       ///< Tilt forward (with Modifier: backward)
  Recline = 1u << 25,    ///< Recline forward (with Modifier: backward)
  Legs = 1u << 26,       ///< Legrests up (with Modifier: down)
  Elevate = 1u << 27,    ///< Seat elevate up (with Modifier: down)
  Footplates = 1u << 28, ///< Footplates up (with Modifier: down)
  Stand = 1u << 29,      ///< Stand up (with Modifier: down)
  // bits 30-31 reserved for future seating functions
};

/// @brief Bits of the Feedback report's "Standard" u32 bitfield (bytes 0..3).
enum class FeedbackBit : uint32_t {
  DriveDisabled = 1u << 0,
  DriveEnabled = 1u << 1,
  ModeDrive = 1u << 2,
  ModeSeating = 1u << 3,
  LeftBlinkerOff = 1u << 4,
  LeftBlinkerOn = 1u << 5,
  RightBlinkerOff = 1u << 6,
  RightBlinkerOn = 1u << 7,
  HeadlightsOff = 1u << 8,
  HeadlightsOn = 1u << 9,
  HazardsOff = 1u << 10,
  HazardsOn = 1u << 11,
  NoMovementRestriction = 1u << 12,
  LimitedSpeed = 1u << 13,
  NoMovement = 1u << 14,
  // bits 15-31 reserved
};

// --- little-endian helpers ---------------------------------------------------
namespace detail {
inline void put_u32_le(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}
inline uint32_t get_u32_le(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
} // namespace detail

/// @brief The Control report (ID 0x01): joystick position + control-flag bitfields.
///        Sent by the app / controller to the wheelchair.
struct ControlReport {
  int8_t x{0};           ///< Lateral: -127 (full left) .. +127 (full right)
  int8_t y{0};           ///< Longitudinal: -127 (full forward) .. +127 (full reverse)
  uint32_t standard1{0}; ///< OR of ControlBit values
  uint32_t standard2{0}; ///< reserved (all bits reserved for future use)
  uint32_t vendor1{0};   ///< vendor-specific; keyed by manufacturer id. Per the spec its
                         ///< bit0 is also a Modifier (a vendor-scope modifier, distinct from
                         ///< ControlBit::Modifier in standard1).
  uint32_t vendor2{0};   ///< vendor-specific

  /// @brief Whether a Control bit is set in `standard1`.
  bool has(ControlBit bit) const { return (standard1 & static_cast<uint32_t>(bit)) != 0; }
  /// @brief Set or clear a Control bit in `standard1`.
  void set(ControlBit bit, bool on = true) {
    if (on)
      standard1 |= static_cast<uint32_t>(bit);
    else
      standard1 &= ~static_cast<uint32_t>(bit);
  }
  /// @brief True if this is a "release" report (all fields zero).
  bool is_release() const {
    return x == 0 && y == 0 && standard1 == 0 && standard2 == 0 && vendor1 == 0 && vendor2 == 0;
  }

  /// @brief Serialize to the 18-byte report payload (no report-id byte).
  std::array<uint8_t, kControlSize> serialize() const {
    std::array<uint8_t, kControlSize> b{};
    b[0] = static_cast<uint8_t>(x);
    b[1] = static_cast<uint8_t>(y);
    detail::put_u32_le(&b[2], standard1);
    detail::put_u32_le(&b[6], standard2);
    detail::put_u32_le(&b[10], vendor1);
    detail::put_u32_le(&b[14], vendor2);
    return b;
  }
  /// @brief Parse an 18-byte payload; std::nullopt if the wrong size.
  static std::optional<ControlReport> parse(std::span<const uint8_t> p) {
    if (p.size() != kControlSize)
      return std::nullopt;
    ControlReport r;
    r.x = static_cast<int8_t>(p[0]);
    r.y = static_cast<int8_t>(p[1]);
    r.standard1 = detail::get_u32_le(&p[2]);
    r.standard2 = detail::get_u32_le(&p[6]);
    r.vendor1 = detail::get_u32_le(&p[10]);
    r.vendor2 = detail::get_u32_le(&p[14]);
    return r;
  }
};

/// @brief The Feedback report (ID 0x02): status flags + speed / velocity / odometer.
///        Sent by the wheelchair to the app / controller.
struct FeedbackReport {
  uint32_t standard{0};       ///< OR of FeedbackBit values
  uint32_t vendor1{0};        ///< vendor-specific
  uint32_t vendor2{0};        ///< vendor-specific
  uint8_t speed{0};           ///< current speed setting 0..15 (0 = unknown)
  uint8_t profile{0};         ///< current drive profile 0..15 (0 = unknown)
  uint8_t velocity_whole{0};  ///< whole mph, 0..15
  uint8_t velocity_tenths{0}; ///< tenths of mph, 0..9 (so 0.0 .. 15.9 mph)
  uint8_t odometer{0};        ///< odometer (u8, units per spec/vendor)

  bool has(FeedbackBit bit) const { return (standard & static_cast<uint32_t>(bit)) != 0; }
  void set(FeedbackBit bit, bool on = true) {
    if (on)
      standard |= static_cast<uint32_t>(bit);
    else
      standard &= ~static_cast<uint32_t>(bit);
  }
  /// @brief Velocity as mph (whole + tenths/10).
  float velocity_mph() const {
    return static_cast<float>(velocity_whole) + static_cast<float>(velocity_tenths) / 10.0f;
  }

  /// @brief Serialize to the 19-byte report payload (no report-id byte).
  std::array<uint8_t, kFeedbackSize> serialize() const {
    std::array<uint8_t, kFeedbackSize> b{};
    detail::put_u32_le(&b[0], standard);
    detail::put_u32_le(&b[4], vendor1);
    detail::put_u32_le(&b[8], vendor2);
    // Byte 12: high nibble = speed, low nibble = profile (each 0..15).
    b[12] = static_cast<uint8_t>(((speed & 0x0F) << 4) | (profile & 0x0F));
    // Byte 13: high nibble = whole mph (0..15), low nibble = tenths (0..9). Clamp
    // tenths to 9 so an out-of-range value can't encode an invalid 10..15 nibble.
    const uint8_t tenths = velocity_tenths > 9 ? 9 : velocity_tenths;
    b[13] = static_cast<uint8_t>(((velocity_whole & 0x0F) << 4) | (tenths & 0x0F));
    b[14] = odometer;
    // bytes 15..18 reserved (left zero)
    return b;
  }
  /// @brief Parse a 19-byte payload; std::nullopt if the wrong size.
  static std::optional<FeedbackReport> parse(std::span<const uint8_t> p) {
    if (p.size() != kFeedbackSize)
      return std::nullopt;
    FeedbackReport r;
    r.standard = detail::get_u32_le(&p[0]);
    r.vendor1 = detail::get_u32_le(&p[4]);
    r.vendor2 = detail::get_u32_le(&p[8]);
    r.speed = static_cast<uint8_t>((p[12] >> 4) & 0x0F);
    r.profile = static_cast<uint8_t>(p[12] & 0x0F);
    r.velocity_whole = static_cast<uint8_t>((p[13] >> 4) & 0x0F);
    r.velocity_tenths = static_cast<uint8_t>(p[13] & 0x0F);
    r.odometer = p[14];
    return r;
  }
};

/// @brief The host's 128-bit identity from a Keepalive Response (ID 0x05), stored
///        big-endian (network byte order) exactly as it appears on the wire. The
///        app should display the full 16-byte UUID; the manufacturer name is
///        supplementary. Bytes 0..1 are the 16-bit manufacturer id (big-endian);
///        bytes 2..15 are RFC 4122 v4 random (byte 6 high nibble 0x4, byte 8 top
///        bits 0b10).
struct HostUuid {
  std::array<uint8_t, kKeepaliveResponseSize> bytes{};

  /// @brief The 16-bit manufacturer id (big-endian in bytes 0..1).
  uint16_t manufacturer_id() const {
    return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
  }

  const std::array<uint8_t, kKeepaliveResponseSize> &serialize() const { return bytes; }
  static std::optional<HostUuid> parse(std::span<const uint8_t> p) {
    if (p.size() != kKeepaliveResponseSize)
      return std::nullopt;
    HostUuid u;
    for (size_t i = 0; i < kKeepaliveResponseSize; ++i)
      u.bytes[i] = p[i];
    return u;
  }
};

/// @brief Registered WDI manufacturer ids (subset; see the spec's registry).
enum class ManufacturerId : uint16_t {
  Unknown = 0x0000,
  LuciMobility = 0x000B,
  LifeDrive = 0x000C,
};

/// @brief Keepalive / timeout timing constants from the spec.
inline constexpr uint32_t kAppKeepaliveIntervalMs = 233; ///< app sends every ~233 ms
inline constexpr uint32_t kHostKeepaliveWindowMs = 257;  ///< host's per-window timeout
inline constexpr uint32_t kHostMissedWindowsToDisconnect =
    3; ///< 3 missed → disconnect + drive-disable

// The HID report descriptor (usage page 0xFF00) lives in wdi_hid.hpp, built with
// the espp hid-rp component. It is only needed by the USB HID transport (BLE
// carries the same reports as GATT characteristics), so it is kept out of this
// dependency-free core.

} // namespace wdi
} // namespace espp
