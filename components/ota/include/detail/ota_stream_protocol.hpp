#pragma once

// espp OTA stream protocol — wire framing for OTA over a raw byte stream.
//
// This header is intentionally free of any ESP-IDF / FreeRTOS dependency so
// that the framing logic (CRC-32, frame building, incremental parsing with
// resynchronization) can be built and unit-tested on a host with nothing more
// than a C++20 standard library. The espp `ota` example composes this framing
// with `espp::Ota` and `espp::UsbDevice` to stream firmware over the USB
// vendor (WebUSB) interface; the `ota_console.html` web app implements the
// exact same framing in JavaScript.
//
// Wire format (all multi-byte fields LITTLE-ENDIAN):
//
//   [magic u16 = 0x4F54 ("OT")][type u8][len u32][payload: len bytes][crc32 u32]
//
//   - magic: the u16 value 0x4F54 ("OT"), transmitted little-endian, so the
//     raw byte sequence on the wire is 0x54 ('T') then 0x4F ('O').
//   - type:  one of the MessageType values below.
//   - len:   payload length in bytes; MUST be <= kMaxPayloadSize (4096). The
//     parser rejects (and resynchronizes past) any frame whose length field
//     exceeds this cap, so a remote-supplied length can never cause unbounded
//     buffering.
//   - crc32: standard zlib CRC-32 (IEEE 802.3: polynomial 0xEDB88320
//     reflected, init 0xFFFFFFFF, final xor 0xFFFFFFFF) computed over
//     magic..payload, i.e. the kHeaderSize (7) header bytes plus the payload.
//     Golden check value: crc32("123456789") == 0xCBF43926.
//
// Message types & payloads (host -> device):
//   0x01 BEGIN  — payload: u32 image_size (0 = unknown / streaming).
//   0x02 DATA   — payload: raw image bytes (1..kMaxPayloadSize per frame).
//   0x03 FINISH — no payload. Validates + activates the received image.
//   0x04 ABORT  — no payload. Discards the in-progress session.
//
// Message types & payloads (device -> host):
//   0x81 OK       — payload: u32 bytes_received so far. Sent in reply to each
//                   successfully-handled BEGIN / DATA / FINISH / ABORT.
//   0x82 ERROR    — payload: u32 code followed by a UTF-8 message.
//   0x83 PROGRESS — payload: u32 written, u32 total (0 if unknown). Optional,
//                   informational; hosts must tolerate (and may ignore) it.
//
// Flow control: the host serializes transactions — it sends one frame and
// waits for the matching OK / ERROR reply before sending the next — so the
// device never needs to buffer more than one frame of image data.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace espp {
namespace detail {
namespace ota_stream {

/// Frame magic: the u16 value 0x4F54 ("OT"); little-endian on the wire, so the
/// first frame byte is 0x54 ('T') and the second is 0x4F ('O').
static constexpr uint16_t kMagic = 0x4F54;
/// First (low) magic byte on the wire.
static constexpr uint8_t kMagicByte0 = static_cast<uint8_t>(kMagic & 0xFF); // 0x54 'T'
/// Second (high) magic byte on the wire.
static constexpr uint8_t kMagicByte1 = static_cast<uint8_t>(kMagic >> 8); // 0x4F 'O'
/// Frame header size: magic (2) + type (1) + len (4).
static constexpr size_t kHeaderSize = 7;
/// Trailing CRC-32 size.
static constexpr size_t kCrcSize = 4;
/// Maximum payload bytes per frame. Frames whose length field exceeds this are
/// rejected and resynchronized past, bounding parser memory usage.
static constexpr size_t kMaxPayloadSize = 4096;
/// Maximum total encoded frame size (header + payload + crc) = 4107 bytes.
static constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;

/// OTA stream protocol message types.
enum class MessageType : uint8_t {
  Begin = 0x01,    ///< host -> device: start a session (payload: u32 image_size, 0 = unknown)
  Data = 0x02,     ///< host -> device: image bytes (payload: raw image data)
  Finish = 0x03,   ///< host -> device: validate + activate the received image (no payload)
  Abort = 0x04,    ///< host -> device: discard the in-progress session (no payload)
  Ok = 0x81,       ///< device -> host: success reply (payload: u32 bytes_received so far)
  Error = 0x82,    ///< device -> host: failure reply (payload: u32 code + utf8 message)
  Progress = 0x83, ///< device -> host: optional progress (payload: u32 written, u32 total)
};

/// @brief Standard zlib CRC-32 (IEEE 802.3; poly 0xEDB88320 reflected, init
///        0xFFFFFFFF, final xor 0xFFFFFFFF).
/// @param data Bytes to checksum.
/// @param crc Running CRC from a previous call (0 to start, matching zlib's
///        crc32(0, ...) convention); chainable across chunks.
/// @return The CRC-32 of the concatenated input.
/// @note Golden check value: crc32 over the ASCII bytes "123456789" == 0xCBF43926.
inline uint32_t crc32(std::span<const uint8_t> data, uint32_t crc = 0) {
  crc = ~crc;
  for (const uint8_t byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; bit++)
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
  }
  return ~crc;
}

/// Append a u16 little-endian to a byte vector.
inline void put_u16(std::vector<uint8_t> &out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

/// Append a u32 little-endian to a byte vector.
inline void put_u32(std::vector<uint8_t> &out, uint32_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

/// Read a u32 little-endian from a byte span (span must hold >= 4 bytes).
inline uint32_t get_u32(std::span<const uint8_t> bytes) {
  return static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
}

/// @brief A complete, CRC-verified protocol frame.
struct Frame {
  MessageType type;             ///< Message type byte (unknown values are passed through).
  std::vector<uint8_t> payload; ///< Payload bytes (may be empty).
};

/// @brief Build an encoded frame: header + payload + CRC-32 over magic..payload.
/// @param type Message type.
/// @param payload Payload bytes; must be <= kMaxPayloadSize.
/// @return The encoded frame bytes, or an empty vector if the payload is too large.
inline std::vector<uint8_t> build_frame(MessageType type, std::span<const uint8_t> payload = {}) {
  if (payload.size() > kMaxPayloadSize)
    return {};
  std::vector<uint8_t> out;
  out.reserve(kHeaderSize + payload.size() + kCrcSize);
  put_u16(out, kMagic);
  out.push_back(static_cast<uint8_t>(type));
  put_u32(out, static_cast<uint32_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  put_u32(out, crc32(std::span<const uint8_t>(out.data(), out.size())));
  return out;
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

/// @brief Incremental frame parser for the OTA stream protocol.
///
/// Feed arbitrary chunks of received bytes (USB bulk transfers, socket reads,
/// single bytes, ...) and it yields the complete, CRC-verified frames they
/// contain. On a bad magic, an oversized length field (> kMaxPayloadSize) or a
/// CRC mismatch it resynchronizes by discarding bytes until the next plausible
/// frame start, so a corrupted stream recovers at the next intact frame.
///
/// Buffering is bounded: because the length field is capped, the parser never
/// retains more than kMaxFrameSize bytes between feed() calls (plus at most the
/// chunk currently being processed), so a remote-supplied length cannot cause
/// memory exhaustion.
class StreamParser {
public:
  /// @brief Feed received bytes to the parser.
  /// @param data Any number of bytes (frames may be split or batched arbitrarily).
  /// @return All complete, CRC-verified frames terminated by these bytes, in order.
  std::vector<Frame> feed(std::span<const uint8_t> data) {
    buffer_.insert(buffer_.end(), data.begin(), data.end());
    std::vector<Frame> frames;
    size_t pos = 0;
    while (true) {
      pos = find_frame_start(pos);
      if (buffer_.size() - pos < kHeaderSize)
        break; // incomplete header; wait for more bytes
      const auto header = std::span<const uint8_t>(buffer_).subspan(pos);
      const uint32_t len = get_u32(header.subspan(3));
      if (len > kMaxPayloadSize) {
        // Reject a remote-supplied length that exceeds the cap and resync one
        // byte past this (bogus) frame start.
        dropped_bytes_++;
        pos++;
        continue;
      }
      const size_t total = kHeaderSize + len + kCrcSize;
      if (buffer_.size() - pos < total)
        break; // incomplete frame; wait for more bytes
      const uint32_t expected = get_u32(header.subspan(kHeaderSize + len));
      const uint32_t actual = crc32(header.first(kHeaderSize + len));
      if (actual != expected) {
        // Corrupt frame; resync one byte past this frame start.
        dropped_bytes_++;
        pos++;
        continue;
      }
      Frame frame{};
      frame.type = static_cast<MessageType>(header[2]);
      frame.payload.assign(header.begin() + kHeaderSize, header.begin() + kHeaderSize + len);
      frames.push_back(std::move(frame));
      pos += total;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
    return frames;
  }

  /// Discard all buffered bytes (e.g. on transport reconnect).
  void reset() { buffer_.clear(); }

  /// Number of bytes currently buffered awaiting frame completion.
  size_t buffered() const { return buffer_.size(); }

  /// Total bytes discarded so far while resynchronizing (diagnostics).
  size_t dropped_bytes() const { return dropped_bytes_; }

protected:
  /// Advance @p pos to the next plausible frame start (the magic byte pair),
  /// counting the skipped bytes as dropped. A trailing lone kMagicByte0 is kept
  /// (it may be the first half of a magic split across chunks).
  size_t find_frame_start(size_t pos) {
    const size_t start = pos;
    while (pos < buffer_.size()) {
      if (buffer_[pos] == kMagicByte0) {
        if (pos + 1 >= buffer_.size() || buffer_[pos + 1] == kMagicByte1)
          break; // found (or possibly-split) magic
      }
      pos++;
    }
    dropped_bytes_ += pos - start;
    return pos;
  }

private:
  std::vector<uint8_t> buffer_;
  size_t dropped_bytes_{0};
};

} // namespace ota_stream
} // namespace detail
} // namespace espp
