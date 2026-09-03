#pragma once

#include <array>
#include <cstdint>

/// \file mcp266_core.hpp
/// \brief Host-buildable, ESP-independent core for the Basicmicro MCP266
///        CANopen mapping: the manufacturer command-object mirror, per-axis
///        object addresses, and the position-PID field-order remap. Pure
///        constexpr so it can be unit-tested on the host.

namespace espp {
namespace detail {
namespace mcp266 {

/// \brief The MCP266 mirrors its packet-serial command set into the
///        manufacturer region of the CANopen object dictionary at index
///        0x2000 + command number. Verified on MCP266 firmware: command 61
///        (set M1 position PID) = 0x203D, 55 (read M1 velocity PID) = 0x2037,
///        24 (read main battery) = 0x2018, 200 (e-stop reset) = 0x20C8.
/// \param command The Basicmicro packet-serial command byte.
/// \return The corresponding manufacturer object index.
inline constexpr uint16_t command_object(uint8_t command) {
  return static_cast<uint16_t>(0x2000 + command);
}

/// \brief Object offset added to a CiA 402 device-profile index (0x60xx) to
///        select a motor axis. M1 is at the standard indices; M2 mirrors them
///        at +0x800 (e.g. controlword 0x6040 -> 0x6840).
/// @{
inline constexpr uint16_t kAxisOffsetM1 = 0x000;
inline constexpr uint16_t kAxisOffsetM2 = 0x800;
/// @}

/// \brief Device-level (non-axis) telemetry / maintenance objects, mirrored
///        from their packet-serial commands.
/// @{
inline constexpr uint16_t kMainBatteryObject = command_object(24); ///< tenths of a volt (u16)
inline constexpr uint16_t kTemperatureObject = command_object(82); ///< tenths of a degree C (u16)
inline constexpr uint16_t kEStopResetObject = command_object(200); ///< write-only
/// @}

/// \brief The manufacturer command objects and CiA 402 offset for one axis.
struct AxisObjects {
  uint16_t object_offset;    ///< 0 for M1, 0x800 for M2 (added to 0x60xx objects).
  uint16_t position_pid_set; ///< Position PID setter (command 61/62), write-only.
  uint16_t position_pid_get; ///< Position PID readback (command 63/64).
  uint16_t drive_duty;       ///< Signed-duty command (32/33).
  uint16_t drive_speed;      ///< Signed-speed command (35/36).
};

/// \brief Objects for motor 1 (the standard axis).
inline constexpr AxisObjects axis_m1() {
  return {kAxisOffsetM1, command_object(61), command_object(63), command_object(32),
          command_object(35)};
}
/// \brief Objects for motor 2 (mirrored at +0x800 / command n+1).
inline constexpr AxisObjects axis_m2() {
  return {kAxisOffsetM2, command_object(62), command_object(64), command_object(33),
          command_object(36)};
}

/// \brief Remap a position-PID record from the readback order to the setter
///        order.
/// \details The readback (commands 63/64) reports
///          [P, I, D, MaxI, Deadzone, MinPos, MaxPos] but the setter
///          (commands 61/62) takes the packet-serial write order
///          [D, P, I, MaxI, Deadzone, MinPos, MaxPos]. Writing the readback
///          array straight back would put P into the D slot and leave P at 0
///          (no loop output). This performs the correct field shuffle.
/// \param readback The seven values as read from the readback object.
/// \return The seven values in setter order.
inline constexpr std::array<int32_t, 7>
position_pid_readback_to_setter(const std::array<int32_t, 7> &readback) {
  return {readback[2], readback[0], readback[1], readback[3],
          readback[4], readback[5], readback[6]};
}

} // namespace mcp266
} // namespace detail
} // namespace espp
