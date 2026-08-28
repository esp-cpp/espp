#pragma once

// espp BLDC haptics USB protocol — message ids + payload helpers layered on the
// espp `ota_stream` framing (magic "OT" + type u8 + len u32 + payload + CRC-32,
// all little-endian; see components/ota/include/detail/ota_stream_protocol.hpp
// for the authoritative framing spec and ../PROTOCOL.md next to this example
// for the full haptics wire protocol).
//
// The message-type space is partitioned so the OTA subset stays byte-compatible
// with the espp `ota` example / ota_console.html web app:
//   0x01..0x04  host -> device  OTA (BEGIN / DATA / FINISH / ABORT)
//   0x10..0x2F  host -> device  haptics commands
//   0x81..0x8F  device -> host  generic + OTA replies (OK / ERROR / PROGRESS)
//   0x90..0xAF  device -> host  haptics replies + telemetry

#include <bit>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include "detail/ota_stream_protocol.hpp"

namespace haptics_proto {

namespace stream = espp::detail::ota_stream;

/// Protocol version reported in the INFO reply.
static constexpr uint8_t kProtocolVersion = 1;

/// Message types carried in the ota_stream frame `type` byte.
enum class Msg : uint8_t {
  // --- OTA subset (identical semantics to the espp ota example) -------------
  OtaBegin = 0x01,  ///< host->dev: u32 image_size (0 = unknown / streaming)
  OtaData = 0x02,   ///< host->dev: raw image bytes (<= 4096 per frame)
  OtaFinish = 0x03, ///< host->dev: validate + activate the received image
  OtaAbort = 0x04,  ///< host->dev: discard the in-progress session
  // --- Haptics commands ------------------------------------------------------
  GetInfo = 0x10,      ///< host->dev: no payload -> Info reply
  GetStatus = 0x11,    ///< host->dev: no payload -> Status reply
  GetModes = 0x12,     ///< host->dev: no payload -> Modes reply
  SetMode = 0x13,      ///< host->dev: u8 mode index -> Ok(index)
  SetPosition = 0x14,  ///< host->dev: i32 detent position -> Ok(clamped position)
  SetEnabled = 0x15,   ///< host->dev: u8 0/1 -> Ok(0/1)
  PlayHaptic = 0x16,   ///< host->dev: f32 strength -> Ok(0)
  SetStreaming = 0x17, ///< host->dev: u8 0/1 + u16 period_ms -> Ok(period_ms)
  // --- Generic / OTA replies -------------------------------------------------
  Ok = 0x81,          ///< dev->host: u32 context-dependent value
  Error = 0x82,       ///< dev->host: u32 code (std::errc) + utf8 message
  OtaProgress = 0x83, ///< dev->host: u32 written + u32 total (informational)
  // --- Haptics replies / telemetry -------------------------------------------
  Info = 0x90,      ///< dev->host: protocol version + firmware description
  Status = 0x91,    ///< dev->host: full status snapshot
  Modes = 0x92,     ///< dev->host: enumeration of the detent presets
  Telemetry = 0x93, ///< dev->host: periodic position/detent frame (streaming)
};

// ---------------------------------------------------------------------------
// Little-endian payload append / read helpers (u16/u32 come from ota_stream).
// ---------------------------------------------------------------------------

using stream::get_u32;
using stream::put_u16;
using stream::put_u32;

inline void put_i32(std::vector<uint8_t> &out, int32_t value) {
  put_u32(out, static_cast<uint32_t>(value));
}

inline void put_f32(std::vector<uint8_t> &out, float value) {
  put_u32(out, std::bit_cast<uint32_t>(value));
}

/// Append a u8-length-prefixed UTF-8 string (truncated to 255 bytes).
inline void put_str(std::vector<uint8_t> &out, std::string_view str) {
  const size_t count = std::min<size_t>(str.size(), 0xFF);
  out.push_back(static_cast<uint8_t>(count));
  out.insert(out.end(), str.begin(), str.begin() + count);
}

inline std::optional<uint16_t> get_u16_at(std::span<const uint8_t> bytes, size_t offset) {
  if (bytes.size() < offset + 2)
    return std::nullopt;
  return static_cast<uint16_t>(bytes[offset]) | (static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

inline std::optional<int32_t> get_i32_at(std::span<const uint8_t> bytes, size_t offset) {
  if (bytes.size() < offset + 4)
    return std::nullopt;
  return static_cast<int32_t>(get_u32(bytes.subspan(offset)));
}

inline std::optional<float> get_f32_at(std::span<const uint8_t> bytes, size_t offset) {
  if (bytes.size() < offset + 4)
    return std::nullopt;
  return std::bit_cast<float>(get_u32(bytes.subspan(offset)));
}

/// Build a frame for any haptics-protocol message type.
inline std::vector<uint8_t> build(Msg type, std::span<const uint8_t> payload = {}) {
  return stream::build_frame(static_cast<stream::MessageType>(type), payload);
}

/// Status flag bits (Status + Telemetry `flags` byte).
namespace flags {
static constexpr uint8_t kEnabled = 1 << 0;   ///< haptic engine running
static constexpr uint8_t kFaulted = 1 << 1;   ///< motor driver fault asserted
static constexpr uint8_t kStreaming = 1 << 2; ///< telemetry streaming active
} // namespace flags

} // namespace haptics_proto
