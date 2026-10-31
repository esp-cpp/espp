#pragma once

#include <cstdint>

// Basicmicro (MCP236 / MCP266 / RoboClaw-family) command-number table -- the
// single source of truth for the packet-serial command bytes.
//
// This lives in the shared `motor_controller` component (which both drivers of
// this hardware family already depend on) because the command NUMBERS are used
// by BOTH transports and must stay in lockstep:
//   * espp::Basicmicro (packet serial) sends `[addr][command][data][CRC16]`.
//   * espp::Mcp266 (CANopen) talks to the same controller, whose firmware
//     mirrors the packet-serial command set into the manufacturer region of the
//     object dictionary at index 0x2000 + command number (see
//     mcp266_core.hpp::command_object). So e.g. "read main battery voltage" is
//     command 24 on serial and object 0x2018 on CAN -- the SAME 24.
// Keeping the numbers here lets mcp266 derive its object indices from these
// named values instead of hardcoding magic numbers that could silently drift
// from the serial driver. It is a dependency-free header (just an enum), so both
// host-buildable cores can include it without pulling in anything else.

namespace espp {
namespace detail {

/// @brief Packet-serial command bytes.
///
/// Every value below was verified against the MCP Series User Manual (sections
/// 2.2.12, 2.2.13, 2.3.1, 2.4.8 and 2.4.9). Commands whose payload layout is
/// not clearly documented in the manual are intentionally omitted.
enum class BasicmicroCommand : uint8_t {
  // -- Compatibility commands (section 2.2.12), payload: one byte 0-127 --
  DriveForwardM1 = 0,   ///< 0 = stop, 127 = full forward
  DriveBackwardsM1 = 1, ///< 0 = stop, 127 = full reverse
  DriveForwardM2 = 4,   ///< 0 = stop, 127 = full forward
  DriveBackwardsM2 = 5, ///< 0 = stop, 127 = full reverse
  DriveM1_7Bit = 6,     ///< 0 = full reverse, 64 = stop, 127 = full forward
  DriveM2_7Bit = 7,     ///< 0 = full reverse, 64 = stop, 127 = full forward
  // -- Encoder commands (section 2.4.8) --
  ReadEncoderM1 = 16,           ///< reply: count (4 bytes), status (1 byte)
  ReadEncoderM2 = 17,           ///< reply: count (4 bytes), status (1 byte)
  ReadEncoderSpeedM1 = 18,      ///< reply: pulses/s (4 bytes), direction (1 byte)
  ReadEncoderSpeedM2 = 19,      ///< reply: pulses/s (4 bytes), direction (1 byte)
  ResetEncoders = 20,           ///< payload: none (write command, CRC appended)
  ReadFirmwareVersion = 21,     ///< reply: string terminated by LF + NUL (<=48 bytes)
  SetEncoderM1 = 22,            ///< payload: value (4 bytes)
  SetEncoderM2 = 23,            ///< payload: value (4 bytes)
  ReadMainBatteryVoltage = 24,  ///< reply: tenths of a volt (2 bytes)
  ReadLogicBatteryVoltage = 25, ///< reply: tenths of a volt (2 bytes)
  SetVelocityPidM1 = 28,        ///< payload: D, P, I (16.16 fixed), QPPS (4 bytes each)
  SetVelocityPidM2 = 29,        ///< payload: D, P, I (16.16 fixed), QPPS (4 bytes each)
  ReadRawSpeedM1 = 30,          ///< reply: counts/s (4 bytes), direction (1 byte)
  ReadRawSpeedM2 = 31,          ///< reply: counts/s (4 bytes), direction (1 byte)
  // -- Advanced motor control (section 2.4.9) --
  DriveM1SignedDuty = 32,              ///< payload: duty (2 bytes, +/-32767)
  DriveM2SignedDuty = 33,              ///< payload: duty (2 bytes, +/-32767)
  DriveM1M2SignedDuty = 34,            ///< payload: dutyM1 (2 bytes), dutyM2 (2 bytes)
  DriveM1SignedSpeed = 35,             ///< payload: speed (4 bytes, qpps)
  DriveM2SignedSpeed = 36,             ///< payload: speed (4 bytes, qpps)
  DriveM1M2SignedSpeed = 37,           ///< payload: speedM1 (4 bytes), speedM2 (4 bytes)
  DriveM1SignedSpeedAccel = 38,        ///< payload: accel (4 bytes), speed (4 bytes)
  DriveM2SignedSpeedAccel = 39,        ///< payload: accel (4 bytes), speed (4 bytes)
  DriveM1M2SignedSpeedAccel = 40,      ///< payload: accel, speedM1, speedM2 (4 bytes each)
  BufferedM1SpeedDistance = 41,        ///< payload: speed, distance (4 bytes each), buffer (1 byte)
  BufferedM2SpeedDistance = 42,        ///< payload: speed, distance (4 bytes each), buffer (1 byte)
  BufferedM1M2SpeedDistance = 43,      ///< payload: speedM1, distM1, speedM2, distM2, buffer
  BufferedM1SpeedAccelDistance = 44,   ///< payload: accel, speed, distance, buffer
  BufferedM2SpeedAccelDistance = 45,   ///< payload: accel, speed, distance, buffer
  BufferedM1M2SpeedAccelDistance = 46, ///< payload: accel, speedM1, distM1, speedM2, distM2, buffer
  ReadBufferLengths = 47,              ///< reply: bufferM1 (1 byte), bufferM2 (1 byte)
  ReadMotorPWMs = 48,                  ///< reply: pwmM1 (2 bytes), pwmM2 (2 bytes), +/-32767
  ReadMotorCurrents = 49,       ///< reply: currentM1 (2 bytes), currentM2 (2 bytes), 10 mA units
  ReadVelocityPidM1 = 55,       ///< reply: P, I, D (16.16 fixed), QPPS (4 bytes each)
  ReadVelocityPidM2 = 56,       ///< reply: P, I, D (16.16 fixed), QPPS (4 bytes each)
  SetMainBatteryVoltages = 57,  ///< payload: min (2 bytes), max (2 bytes), tenths of a volt
  SetLogicBatteryVoltages = 58, ///< payload: min (2 bytes), max (2 bytes), tenths of a volt
  ReadMainBatteryVoltageSettings = 59,  ///< reply: min (2 bytes), max (2 bytes)
  ReadLogicBatteryVoltageSettings = 60, ///< reply: min (2 bytes), max (2 bytes)
  SetPositionPidM1 =
      61, ///< payload: D, P, I (scaled 1024), MaxI, Deadzone, MinPos, MaxPos (4 each)
  SetPositionPidM2 =
      62, ///< payload: D, P, I (scaled 1024), MaxI, Deadzone, MinPos, MaxPos (4 each)
  ReadPositionPidM1 = 63, ///< reply: P, I, D (scaled 1024), MaxI, Deadzone, MinPos, MaxPos (4 each)
  ReadPositionPidM2 = 64, ///< reply: P, I, D (scaled 1024), MaxI, Deadzone, MinPos, MaxPos (4 each)
  SetM1DefaultDutyAccel = 68, ///< payload: accel (4 bytes)
  SetM2DefaultDutyAccel = 69, ///< payload: accel (4 bytes)
  ReadEncoderCounters = 78,   ///< reply: encM1 (4 bytes), encM2 (4 bytes)
  ReadISpeedCounters = 79,    ///< reply: ispeedM1 (4 bytes), ispeedM2 (4 bytes)
  RestoreDefaults = 80,       ///< payload: none (write command, CRC appended)
  ReadDefaultDutyAccels = 81, ///< reply: accelM1 (4 bytes), accelM2 (4 bytes)
  ReadTemperature = 82,       ///< reply: tenths of a degree (2 bytes)
  ReadTemperature2 = 83,      ///< reply: tenths of a degree (2 bytes), supported units only
  // -- Status / configuration (section 2.3.1) --
  ReadStatus = 90,            ///< reply: status bit mask (see BasicmicroStatus)
  ReadEncoderModes = 91,      ///< reply: encM1 mode (1 byte), encM2 mode (1 byte)
  SetEncoderModeM1 = 92,      ///< payload: pin/mode (1 byte)
  SetEncoderModeM2 = 93,      ///< payload: pin/mode (1 byte)
  WriteSettingsToEeprom = 94, ///< no payload and no CRC on the request (per manual), ACK reply
  EStopReset = 200,           ///< payload: none (write command, CRC appended)
};

} // namespace detail
} // namespace espp
