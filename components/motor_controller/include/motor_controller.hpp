#pragma once

#include <concepts>
#include <cstdint>
#include <system_error>

#include "format.hpp"

namespace espp {

/// \brief Motor channel selector shared by the espp motor-controller drivers.
/// \details M1 / M2 are the two output channels of a dual-channel controller
///          (e.g. a Basicmicro MCP236/MCP266). Drivers that model this hardware
///          family select a channel with this type so generic code can drive
///          either transport (packet-serial espp::Basicmicro or CANopen
///          espp::Mcp266) uniformly.
enum class MotorAxis : uint8_t { M1 = 0, M2 = 1 };

/// \brief Compile-time contract for a dual-channel motor controller.
/// \details Both espp::Basicmicro (packet serial) and espp::Mcp266 (CANopen)
///          model the same MCP236/266 hardware and satisfy this concept, so
///          generic code can command either by axis. Every operation follows the
///          espp convention: it returns \c true on success and sets \p ec on
///          failure (leaving it cleared on success). Units are encoder counts
///          (position), counts/s (velocity / "qpps"), a signed duty (±32767),
///          volts, and degrees Celsius.
/// \note Conformance is about the API surface, not that every command is active
///       on every device in its current mode: a controller may accept a duty /
///       speed command it cannot act on (e.g. Mcp266's manufacturer speed/duty
///       mirror is inert on firmware where only CiA 402 position mode drives).
///       Position control and per-device configuration are intentionally NOT
///       part of this common surface -- they differ too much between transports.
template <typename T>
concept MotorController = requires(T t, MotorAxis axis, int16_t duty, int32_t qpps, int32_t count,
                                   float value, std::error_code &ec) {
  /// Open-loop signed duty on one channel.
  { t.drive_duty(axis, duty, ec) } -> std::same_as<bool>;
  /// Closed-loop signed speed (counts/s) on one channel.
  { t.drive_speed(axis, qpps, ec) } -> std::same_as<bool>;
  /// Read the signed encoder count of one channel.
  { t.read_encoder(axis, count, ec) } -> std::same_as<bool>;
  /// Read the signed encoder speed (counts/s) of one channel.
  { t.read_speed(axis, qpps, ec) } -> std::same_as<bool>;
  /// Clear a latched emergency stop.
  { t.reset_estop(ec) } -> std::same_as<bool>;
  /// Read the main battery / supply voltage (volts).
  { t.read_main_battery_voltage(value, ec) } -> std::same_as<bool>;
  /// Read the board temperature (degrees Celsius).
  { t.read_temperature(value, ec) } -> std::same_as<bool>;
};

} // namespace espp

// @brief fmt formatter for espp::MotorAxis (prints "M1" / "M2").
template <> struct fmt::formatter<espp::MotorAxis> : fmt::formatter<std::string_view> {
  template <typename FormatContext> auto format(espp::MotorAxis axis, FormatContext &ctx) const {
    return fmt::formatter<std::string_view>::format(axis == espp::MotorAxis::M1 ? "M1" : "M2", ctx);
  }
};
