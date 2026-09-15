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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <vector>

// The BasicmicroCommand table lives in the shared motor_controller component so
// the mcp266 CANopen driver (which mirrors these commands at object 0x2000 + n)
// can derive its object indices from the same source of truth.
#include "basicmicro_commands.hpp"

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

/// Position PID gains are transferred scaled by 1024 (Basicmicro reference
/// library convention for the position loop; commands 61-64). The velocity
/// loop uses the separate kBasicmicroPidScale above.
static constexpr float kBasicmicroPositionPidScale = 1024.0f;

// BasicmicroCommand (the packet-serial command-number table) is defined in
// basicmicro_commands.hpp, included above -- it is shared with the mcp266
// CANopen driver so the two cannot drift.

/// @brief Unit status bit masks returned by BasicmicroCommand::ReadStatus
///        (manual command 90). Current MCP firmware returns a 32-bit status
///        word (read via Read4 in the official library); the manual documents
///        the masks below, which occupy the low 16 bits.
enum class BasicmicroStatus : uint32_t {
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

/// Convert a floating-point PID gain to the controller's fixed-point wire
/// representation (multiply by \p scale). Rounds to nearest rather than
/// truncating (truncation biases every gain downward by up to ~1 LSB), and
/// clamps to the non-negative uint32_t range: PID gains are non-negative on
/// these controllers, and a raw static_cast of a negative product to uint32_t
/// would silently wrap to a huge value. Non-finite or out-of-range inputs are
/// saturated rather than fed to the rounding function (std::llround of a value
/// outside long long, or of inf/NaN, is undefined).
inline uint32_t scale_pid_gain(float gain, float scale) {
  if (!(gain > 0.0f)) { // false for <= 0 and for NaN
    return 0;
  }
  const double scaled = static_cast<double>(gain) * static_cast<double>(scale);
  // Compare before rounding: a scaled value at/above UINT32_MAX (or +inf) must
  // not reach std::llround, whose result is undefined outside long long's range
  // and for non-finite inputs.
  if (!(scaled < 4294967296.0)) { // 2^32; also false for +inf and NaN
    return UINT32_MAX;
  }
  const long long rounded = std::llround(scaled);
  if (rounded <= 0) {
    return 0;
  }
  if (rounded > static_cast<long long>(UINT32_MAX)) {
    return UINT32_MAX;
  }
  return static_cast<uint32_t>(rounded);
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
