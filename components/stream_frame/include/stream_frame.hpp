#pragma once

// espp stream-frame codec — a tiny, dependency-free wire framing for carrying
// typed, length-delimited, CRC-verified messages over any raw byte stream (USB
// bulk / vendor, CDC, TCP/UDP sockets, UART, ...).
//
// This header deliberately has NO ESP-IDF / FreeRTOS dependency so the framing
// (CRC-32, frame building, incremental parsing with resynchronization) builds
// and unit-tests on a host with nothing more than a C++20 standard library.
//
// It is the shared substrate under several espp protocols: the OTA stream
// protocol (`espp::detail::ota_stream`, which layers its MessageType enum and
// make_*/parse_* helpers on top), the crash-dump service, and — via
// `espp::Dispatcher` — any number of independent protocols multiplexed over one
// stream (routed by the message-type byte's module id). Each protocol owns a
// disjoint range of the u8 `type` space; unknown types are ignored by parsers
// that do not recognize them, so multiple protocols coexist on one stream.
//
// Wire format (all multi-byte fields LITTLE-ENDIAN):
//
//   [magic u16 = 0x4F54 ("OT")][type u8][len u32][payload: len bytes][crc32 u32]
//
//   - magic: the u16 value 0x4F54 ("OT"), transmitted little-endian, so the raw
//     byte sequence on the wire is 0x54 ('T') then 0x4F ('O').
//   - type:  an application message-type byte (see espp::Dispatcher for the
//     module-id convention that lets protocols share the byte space).
//   - len:   payload length in bytes; MUST be <= kMaxPayloadSize (4096). The
//     parser rejects (and resynchronizes past) any frame whose length field
//     exceeds this cap, so a remote-supplied length can never cause unbounded
//     buffering.
//   - crc32: standard zlib CRC-32 (IEEE 802.3: polynomial 0xEDB88320 reflected,
//     init 0xFFFFFFFF, final xor 0xFFFFFFFF) computed over magic..payload, i.e.
//     the kHeaderSize (7) header bytes plus the payload.
//     Golden check value: crc32("123456789") == 0xCBF43926.

#include <cstdint>
#include <span>
#include <vector>

namespace espp {
namespace stream_frame {

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

/// Read a u16 little-endian from a byte span (span must hold >= 2 bytes).
inline uint16_t get_u16(std::span<const uint8_t> bytes) {
  return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) |
                               (static_cast<uint16_t>(bytes[1]) << 8));
}

/// Read a u32 little-endian from a byte span (span must hold >= 4 bytes).
inline uint32_t get_u32(std::span<const uint8_t> bytes) {
  return static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
}

/// @brief A complete, CRC-verified protocol frame.
///
/// `type` is the raw message-type byte; each protocol interprets its own range
/// of the byte space (and, with espp::Dispatcher, its module id).
struct Frame {
  uint8_t type;                 ///< Message type byte (unknown values are passed through).
  std::vector<uint8_t> payload; ///< Payload bytes (may be empty).
};

/// @brief Build an encoded frame: header + payload + CRC-32 over magic..payload.
/// @param type Message type byte.
/// @param payload Payload bytes; must be <= kMaxPayloadSize.
/// @return The encoded frame bytes, or an empty vector if the payload is too large.
inline std::vector<uint8_t> build_frame(uint8_t type, std::span<const uint8_t> payload = {}) {
  if (payload.size() > kMaxPayloadSize)
    return {};
  std::vector<uint8_t> out;
  out.reserve(kHeaderSize + payload.size() + kCrcSize);
  put_u16(out, kMagic);
  out.push_back(type);
  put_u32(out, static_cast<uint32_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  put_u32(out, crc32(std::span<const uint8_t>(out.data(), out.size())));
  return out;
}

/// @brief Incremental frame parser for the stream-frame protocol.
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
      frame.type = header[2];
      frame.payload.assign(header.begin() + kHeaderSize, header.begin() + kHeaderSize + len);
      frames.push_back(std::move(frame));
      pos += total;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
    return frames;
  }

  /// Discard all buffered bytes (e.g. on transport reconnect or RX overflow).
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

} // namespace stream_frame
} // namespace espp
