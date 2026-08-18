#pragma once

// Basicmicro (MCP236 / MCP266 / RoboClaw-family) PACKET SERIAL protocol — wire
// core.
//
// This header is intentionally free of any ESP-IDF / FreeRTOS dependency so
// that the wire logic (CRC16, packet building, reply validation, big-endian
// type codecs) can be built and unit-tested on a host with nothing more than a
// C++20 standard library. The `espp::Basicmicro` component composes this core
// together with `espp::BaseComponent` for logging and adds the transaction
// (write / read+timeout) layer on top of user-provided I/O functions.
//
// Wire format (MCP Series User Manual, section 2.2 "Packet Serial Mode"):
//  - Write commands:  [Address, Command, Data..., CRC16(2 bytes)]
//                     and the controller replies with a single 0xFF ACK byte
//                     (nothing at all is sent back if the packet was invalid).
//  - Read commands:   [Address, Command] (no CRC appended to the request);
//                     the controller replies with the data bytes followed by a
//                     CRC16 computed over the SENT Address and Command bytes
//                     plus all of the reply data bytes (section 2.2.7).
//  - All multi-byte values are big-endian ("high byte first", section 2.2.9),
//    including the CRC16 itself.
//  - Addresses range from 0x80 to 0x87 (section 2.2.2).
//  - CRC16 is the CCITT/XModem variant: polynomial 0x1021, initial value 0,
//    not reflected (section 2.2.6 prints the reference C implementation which
//    is replicated byte-for-byte below).
//  - Packet timeout: a >=10 ms gap between bytes makes the controller discard
//    the partial packet (section 2.2.4), so a >=10 ms receive timeout doubles
//    as the error-recovery mechanism — by the time a reply times out, the
//    controller's packet buffer has already been cleared automatically.

#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <vector>

namespace espp {
namespace detail {

/// The ACK byte returned by the controller for every valid write command.
static constexpr uint8_t kBasicmicroAck = 0xFF;

/// Valid packet-serial address range (manual section 2.2.2).
static constexpr uint8_t kBasicmicroMinAddress = 0x80;
/// Valid packet-serial address range (manual section 2.2.2).
static constexpr uint8_t kBasicmicroMaxAddress = 0x87;

/// A >=10 ms inter-byte gap clears the controller's packet buffer (manual
/// section 2.2.4). Receive timeouts should therefore be at least this long so
/// that a timed-out transaction leaves the controller ready for a new packet.
static constexpr int kBasicmicroPacketTimeoutMs = 10;

/// Velocity PID gains are transferred as 16.16 fixed point (scaled by 65536).
/// The manual (commands 28/29) lists the defaults as P=0x00010000, I=0x00008000
/// and D=0x00004000, i.e. P=1.0, I=0.5, D=0.25.
static constexpr float kBasicmicroPidScale = 65536.0f;

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
  SetM1DefaultDutyAccel = 68,           ///< payload: accel (4 bytes)
  SetM2DefaultDutyAccel = 69,           ///< payload: accel (4 bytes)
  ReadEncoderCounters = 78,             ///< reply: encM1 (4 bytes), encM2 (4 bytes)
  ReadISpeedCounters = 79,              ///< reply: ispeedM1 (4 bytes), ispeedM2 (4 bytes)
  RestoreDefaults = 80,                 ///< payload: none (write command, CRC appended)
  ReadDefaultDutyAccels = 81,           ///< reply: accelM1 (4 bytes), accelM2 (4 bytes)
  ReadTemperature = 82,                 ///< reply: tenths of a degree (2 bytes)
  ReadTemperature2 = 83, ///< reply: tenths of a degree (2 bytes), supported units only
  // -- Status / configuration (section 2.3.1) --
  ReadStatus = 90,            ///< reply: status bit mask (see BasicmicroStatus)
  ReadEncoderModes = 91,      ///< reply: encM1 mode (1 byte), encM2 mode (1 byte)
  SetEncoderModeM1 = 92,      ///< payload: pin/mode (1 byte)
  SetEncoderModeM2 = 93,      ///< payload: pin/mode (1 byte)
  WriteSettingsToEeprom = 94, ///< no payload and no CRC on the request (per manual), ACK reply
  EStopReset = 200,           ///< payload: none (write command, CRC appended)
};

/// @brief Unit status bit masks returned by BasicmicroCommand::ReadStatus
///        (manual command 90).
enum class BasicmicroStatus : uint16_t {
  Normal = 0x0000,
  M1OverCurrentWarning = 0x0001,
  M2OverCurrentWarning = 0x0002,
  EStop = 0x0004,
  TemperatureError = 0x0008,
  Temperature2Error = 0x0010,
  MainBatteryHighError = 0x0020,
  LogicBatteryHighError = 0x0040,
  LogicBatteryLowError = 0x0080,
  MainBatteryHighWarning = 0x0400,
  MainBatteryLowWarning = 0x0800,
  TemperatureWarning = 0x1000,
  Temperature2Warning = 0x2000,
};

/// Fold a single byte through the running CRC16 remainder. This replicates the
/// reference implementation printed in manual section 2.2.6 (CRC-16/XMODEM:
/// polynomial 0x1021, initial value 0, non-reflected, MSB first).
inline uint16_t basicmicro_crc16_byte(uint16_t crc, uint8_t val) {
  crc ^= static_cast<uint16_t>(static_cast<uint16_t>(val) << 8);
  for (int i = 0; i < 8; i++)
    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                         : static_cast<uint16_t>(crc << 1);
  return crc;
}

/// CRC16 over a buffer. The default initial value (0) matches the manual's
/// reference implementation; pass a previous remainder to continue a running
/// CRC (used to seed reply validation with the sent address + command bytes).
inline uint16_t basicmicro_crc16(std::span<const uint8_t> data, uint16_t init = 0) {
  return std::accumulate(data.begin(), data.end(), init, basicmicro_crc16_byte);
}

// --- big-endian codec helpers ("high byte first", manual section 2.2.9) ---

/// Append a single byte.
inline void append_u8(std::vector<uint8_t> &v, uint8_t val) { v.push_back(val); }

/// Append a 16-bit value, high byte first.
inline void append_u16_be(std::vector<uint8_t> &v, uint16_t val) {
  v.push_back(static_cast<uint8_t>(val >> 8));
  v.push_back(static_cast<uint8_t>(val & 0xFF));
}

/// Append a 32-bit value, high byte first.
inline void append_u32_be(std::vector<uint8_t> &v, uint32_t val) {
  v.push_back(static_cast<uint8_t>(val >> 24));
  v.push_back(static_cast<uint8_t>((val >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>(val & 0xFF));
}

/// Append a signed 16-bit value (two's complement), high byte first.
inline void append_i16_be(std::vector<uint8_t> &v, int16_t val) {
  append_u16_be(v, static_cast<uint16_t>(val));
}

/// Append a signed 32-bit value (two's complement), high byte first.
inline void append_i32_be(std::vector<uint8_t> &v, int32_t val) {
  append_u32_be(v, static_cast<uint32_t>(val));
}

/// Read a 16-bit big-endian value at byte offset @p off. The caller must
/// ensure the span holds at least off+2 bytes.
inline uint16_t read_u16_be(std::span<const uint8_t> s, size_t off) {
  return static_cast<uint16_t>((static_cast<uint16_t>(s[off]) << 8) |
                               static_cast<uint16_t>(s[off + 1]));
}

/// Read a 32-bit big-endian value at byte offset @p off. The caller must
/// ensure the span holds at least off+4 bytes.
inline uint32_t read_u32_be(std::span<const uint8_t> s, size_t off) {
  return (static_cast<uint32_t>(s[off]) << 24) | (static_cast<uint32_t>(s[off + 1]) << 16) |
         (static_cast<uint32_t>(s[off + 2]) << 8) | static_cast<uint32_t>(s[off + 3]);
}

/// Read a signed 16-bit big-endian value (two's complement) at byte offset @p off.
inline int16_t read_i16_be(std::span<const uint8_t> s, size_t off) {
  return static_cast<int16_t>(read_u16_be(s, off));
}

/// Read a signed 32-bit big-endian value (two's complement) at byte offset @p off.
inline int32_t read_i32_be(std::span<const uint8_t> s, size_t off) {
  return static_cast<int32_t>(read_u32_be(s, off));
}

// --- packet building / reply validation -----------------------------------

/// @brief Build a write-command packet: [Address, Command, payload..., CRC16].
///        The CRC16 covers the address, command and payload bytes and is
///        appended high byte first.
/// @param address Controller address (0x80 - 0x87).
/// @param command Command byte.
/// @param payload Command data bytes (may be empty, e.g. ResetEncoders).
/// @return The complete packet, ready to transmit.
inline std::vector<uint8_t> build_write_packet(uint8_t address, uint8_t command,
                                               std::span<const uint8_t> payload = {}) {
  std::vector<uint8_t> pkt;
  pkt.reserve(payload.size() + 4);
  pkt.push_back(address);
  pkt.push_back(command);
  pkt.insert(pkt.end(), payload.begin(), payload.end());
  append_u16_be(pkt, basicmicro_crc16(pkt));
  return pkt;
}

/// @brief Build a read-command request: [Address, Command]. Read requests
///        carry no CRC (the CRC of the reply is instead seeded with these two
///        bytes, see validate_reply()).
inline std::vector<uint8_t> build_read_request(uint8_t address, uint8_t command) {
  return {address, command};
}

/// @brief Validate a read-command reply.
///
/// Per manual section 2.2.7 the reply CRC16 is computed over the SENT address
/// and command bytes followed by all reply data bytes, and is transmitted high
/// byte first as the last two bytes of the reply.
/// @param address The address byte that was sent.
/// @param command The command byte that was sent.
/// @param reply The full reply: data bytes followed by the 2 CRC bytes.
/// @return True if the reply is at least 2 bytes and its CRC matches.
inline bool validate_reply(uint8_t address, uint8_t command, std::span<const uint8_t> reply) {
  if (reply.size() < 2)
    return false;
  const uint8_t header[] = {address, command};
  uint16_t crc = basicmicro_crc16(header);
  crc = basicmicro_crc16(reply.first(reply.size() - 2), crc);
  return crc == read_u16_be(reply, reply.size() - 2);
}

} // namespace detail
} // namespace espp
