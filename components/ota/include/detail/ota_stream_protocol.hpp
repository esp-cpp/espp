#pragma once

// espp OTA stream protocol — wire framing for OTA over a raw byte stream.
//
// The generic frame codec (magic / type / len / crc, CRC-32, incremental
// resynchronizing StreamParser) now lives in the dependency-free
// `stream_frame` component (espp::stream_frame). This header layers the
// OTA-specific message-type enum, frame builders (make_*) and payload parsers
// (parse_*) on top, and re-exports the generic pieces under the historical
// `espp::detail::ota_stream` namespace so existing OTA/host code keeps working.
//
// Like stream_frame it stays free of any ESP-IDF / FreeRTOS dependency, so the
// framing builds and unit-tests on a host. The espp `ota` example composes it
// with `espp::Ota`, `espp::Dispatcher` and `espp::UsbDevice` to stream firmware
// over the USB vendor (WebUSB) interface; `ota_console.html` implements the
// exact same framing in JavaScript.
//
// Wire format (all multi-byte fields LITTLE-ENDIAN):
//
//   [magic u16 = 0x4F54 ("OT")][type u8][len u32][payload: len bytes][crc32 u32]
//
// Message types & payloads (host -> device):
//   0x01 BEGIN  — payload: u32 image_size (0 = unknown / streaming).
//   0x02 DATA   — payload: raw image bytes (1..kMaxPayloadSize per frame).
//   0x03 FINISH — no payload. Validates + activates the received image.
//   0x04 ABORT  — no payload. Discards the in-progress session.
//
// Message types & payloads (device -> host):
//   0x81 OK       — payload: u32 bytes_received so far.
//   0x82 ERROR    — payload: u32 code followed by a UTF-8 message.
//   0x83 PROGRESS — payload: u32 written, u32 total (0 if unknown). Optional.
//
// The OTA opcodes occupy module id 0 (high nibble 0); see espp::Dispatcher.
//
// Flow control: the host serializes transactions — it sends one frame and waits
// for the matching OK / ERROR reply before sending the next — so the device
// never needs to buffer more than one frame of image data.

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "stream_frame.hpp"

namespace espp {
namespace detail {
namespace ota_stream {

// --- Re-export the generic stream_frame codec under the historical name ------
using espp::stream_frame::kCrcSize;
using espp::stream_frame::kHeaderSize;
using espp::stream_frame::kMagic;
using espp::stream_frame::kMagicByte0;
using espp::stream_frame::kMagicByte1;
using espp::stream_frame::kMaxFrameSize;
using espp::stream_frame::kMaxPayloadSize;

using espp::stream_frame::crc32;
using espp::stream_frame::get_u16;
using espp::stream_frame::get_u32;
using espp::stream_frame::put_u16;
using espp::stream_frame::put_u32;

using espp::stream_frame::Frame;
using espp::stream_frame::StreamParser;

/// OTA occupies dispatcher module id 0.
static constexpr uint8_t kModule = 0;

/// OTA stream protocol message types (the frame `type` field within module 0).
/// Requests are host->device (frame flag reply=0); replies are device->host
/// (reply=1). `type` alone identifies the message; the reply flag is the generic
/// direction hint.
enum class MessageType : uint8_t {
  Begin = 0x01,    ///< host -> device: start a session (payload: u32 image_size, 0 = unknown)
  Data = 0x02,     ///< host -> device: image bytes (payload: raw image data)
  Finish = 0x03,   ///< host -> device: validate + activate the received image (no payload)
  Abort = 0x04,    ///< host -> device: discard the in-progress session (no payload)
  Ok = 0x05,       ///< device -> host: success reply (payload: u32 bytes_received so far)
  Error = 0x06,    ///< device -> host: failure reply (payload: u32 code + utf8 message)
  Progress = 0x07, ///< device -> host: optional progress (payload: u32 written, u32 total)
};

/// Whether a message type is a device->host reply (sets the frame reply flag).
inline bool is_reply(MessageType type) {
  return type == MessageType::Ok || type == MessageType::Error || type == MessageType::Progress;
}

/// @brief Build an encoded OTA frame (typed overload of stream_frame::build_frame).
/// @param type OTA message type.
/// @param payload Payload bytes; must be <= kMaxPayloadSize.
/// @return The encoded frame bytes, or an empty vector if the payload is too large.
inline std::vector<uint8_t> build_frame(MessageType type, std::span<const uint8_t> payload = {}) {
  return espp::stream_frame::build_frame(is_reply(type), kModule, static_cast<uint8_t>(type),
                                         payload);
}

/// Build a BEGIN frame (image_size in bytes, 0 = unknown / streaming).
inline std::vector<uint8_t> make_begin(uint32_t image_size) {
  std::vector<uint8_t> payload;
  put_u32(payload, image_size);
  return build_frame(MessageType::Begin, payload);
}

/// Build a DATA frame carrying up to kMaxPayloadSize image bytes.
inline std::vector<uint8_t> make_data(std::span<const uint8_t> data) {
  return build_frame(MessageType::Data, data);
}

/// Build a FINISH frame (no payload).
inline std::vector<uint8_t> make_finish() { return build_frame(MessageType::Finish); }

/// Build an ABORT frame (no payload).
inline std::vector<uint8_t> make_abort() { return build_frame(MessageType::Abort); }

/// Build an OK reply (bytes_received so far).
inline std::vector<uint8_t> make_ok(uint32_t bytes_received) {
  std::vector<uint8_t> payload;
  put_u32(payload, bytes_received);
  return build_frame(MessageType::Ok, payload);
}

/// Build an ERROR reply (u32 code + UTF-8 message; the message is truncated if
/// it would overflow the maximum payload size).
inline std::vector<uint8_t> make_error(uint32_t code, std::string_view message) {
  std::vector<uint8_t> payload;
  put_u32(payload, code);
  const size_t max_message = kMaxPayloadSize - payload.size();
  const size_t count = std::min(message.size(), max_message);
  payload.insert(payload.end(), message.begin(), message.begin() + count);
  return build_frame(MessageType::Error, payload);
}

/// Build a PROGRESS reply (bytes written so far, total expected — 0 if unknown).
inline std::vector<uint8_t> make_progress(uint32_t written, uint32_t total) {
  std::vector<uint8_t> payload;
  put_u32(payload, written);
  put_u32(payload, total);
  return build_frame(MessageType::Progress, payload);
}

/// Parse the single-u32 payload of a BEGIN (image_size) or OK (bytes_received)
/// frame; returns std::nullopt if the payload is not exactly 4 bytes.
inline std::optional<uint32_t> parse_u32_payload(const Frame &frame) {
  if (frame.payload.size() != 4)
    return std::nullopt;
  return get_u32(frame.payload);
}

/// Decoded ERROR reply payload.
struct ErrorInfo {
  uint32_t code;       ///< Numeric error code (the device uses std::errc values).
  std::string message; ///< Human-readable UTF-8 message (may be empty).
};

/// Parse an ERROR frame payload; returns std::nullopt if it is shorter than the
/// mandatory 4-byte code.
inline std::optional<ErrorInfo> parse_error(const Frame &frame) {
  if (frame.payload.size() < 4)
    return std::nullopt;
  ErrorInfo info{};
  info.code = get_u32(frame.payload);
  info.message.assign(frame.payload.begin() + 4, frame.payload.end());
  return info;
}

/// Decoded PROGRESS reply payload.
struct ProgressInfo {
  uint32_t written; ///< Bytes written to the update partition so far.
  uint32_t total;   ///< Total expected bytes (0 if unknown).
};

/// Parse a PROGRESS frame payload; returns std::nullopt if it is not exactly 8 bytes.
inline std::optional<ProgressInfo> parse_progress(const Frame &frame) {
  if (frame.payload.size() != 8)
    return std::nullopt;
  ProgressInfo info{};
  info.written = get_u32(frame.payload);
  info.total = get_u32(std::span<const uint8_t>(frame.payload).subspan(4));
  return info;
}

} // namespace ota_stream
} // namespace detail
} // namespace espp
