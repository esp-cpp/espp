#pragma once

// espp stream-frame codec (v2) — a tiny, dependency-free wire framing for
// carrying typed, length-delimited, CRC-verified messages over any raw byte
// stream (USB bulk / vendor, CDC, TCP/UDP sockets, UART, ...).
//
// This header deliberately has NO ESP-IDF / FreeRTOS dependency so the framing
// (CRC-32, frame building, incremental parsing with resynchronization) builds
// and unit-tests on a host with nothing more than a C++20 standard library.
//
// It is the shared substrate under several espp protocols (OTA, crash dump, a
// CAN bridge, ...) and — via espp::Dispatcher — any number of independent
// protocols multiplexed over one stream, routed by the frame's `module` id.
//
// Wire format (v2, all multi-byte fields LITTLE-ENDIAN):
//
//   [magic u16 = 0x4F54 ("OT")][flags u8][module u8][type u8]
//        {[correlation u16] present iff flags bit1}[len u32]
//        [payload: len bytes][crc32 u32]
//
//   - magic:  the u16 value 0x4F54 ("OT"), transmitted little-endian, so the
//             raw bytes on the wire are 0x54 ('T') then 0x4F ('O').
//   - flags:  bit0 = reply — the frame's DIRECTION in a request/response
//             exchange: 0 = request (initiator -> responder), 1 = response or
//             unsolicited event (responder -> initiator). Which physical side is
//             the "initiator" depends on the protocol's roles: a device that is
//             a request responder (e.g. the espp coredump / CAN examples, driven
//             by a browser) reads requests and sends responses; a device that
//             INITIATES a request/response exchange with an external peer sends
//             requests and reads the responses. bit1 = correlation present (see
//             below); bits 2..3 reserved for future optional header fields;
//             bits 4..7 = format version (currently 1).
//   - module: routing/protocol id (0..255). espp::Dispatcher routes on this.
//   - type:   the message / transaction type within the module (0..255). See
//             the Transaction enum for the recommended standard values; a
//             protocol may otherwise define its own type values, and may carry
//             a finer opcode as the first payload byte (Transaction::Custom).
//   - correlation: OPTIONAL u16, present only when flags bit1 is set. A
//             protocol-defined correlation / sequence id, opaque to the codec —
//             use it to match a response to the request it answers when more
//             than one request may be outstanding (one-request-in-flight
//             protocols don't need it and leave the flag clear). It sits in the
//             header, so it is covered by the CRC and is not part of `payload`.
//   - len:    payload length in bytes; MUST be <= kMaxPayloadSize (4096). The
//             parser rejects (and resynchronizes past) any oversized length.
//   - crc32:  standard zlib CRC-32 (IEEE 802.3: poly 0xEDB88320 reflected,
//             init 0xFFFFFFFF, final xor 0xFFFFFFFF) over magic..payload, i.e.
//             the whole header (including the optional correlation) + payload.
//             Golden check value: crc32("123456789") == 0xCBF43926.
//
// Optional header fields are gated by `flags` bits so they cost nothing (and are
// byte-identical to a frame without them) when absent — and new optional fields
// can be added under bits 2..3 later WITHOUT another breaking framing change.
//
// NOTE: this is a breaking change from the v1 single-`type`-byte format; v1 had
// no separate module/flags fields (the module was the high nibble of `type` and
// the reply bit was 0x80). All espp protocols and their web apps use v2.

#include <cstdint>
#include <optional>
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
/// Current frame format version (carried in the high nibble of `flags`).
static constexpr uint8_t kVersion = 1;
/// Base header size with no optional fields: magic (2) + flags (1) + module (1)
/// + type (1) + len (4).
static constexpr size_t kHeaderSize = 9;
/// Size of the optional correlation-id field (present iff kFlagCorrelation).
static constexpr size_t kCorrelationSize = 2;
/// Largest possible header (all optional fields present).
static constexpr size_t kMaxHeaderSize = kHeaderSize + kCorrelationSize;
/// Trailing CRC-32 size.
static constexpr size_t kCrcSize = 4;
/// Maximum payload bytes per frame. Frames whose length field exceeds this are
/// rejected and resynchronized past, bounding parser memory usage.
static constexpr size_t kMaxPayloadSize = 4096;
/// Maximum total encoded frame size (largest header + payload + crc).
static constexpr size_t kMaxFrameSize = kMaxHeaderSize + kMaxPayloadSize + kCrcSize;

/// `flags` bit0: this frame is a response / event (responder->initiator) rather
/// than a request (initiator->responder).
static constexpr uint8_t kFlagReply = 0x01;
/// `flags` bit1: the header carries a u16 correlation / sequence id (after type).
static constexpr uint8_t kFlagCorrelation = 0x02;

/// Extract the format version from a `flags` byte (high nibble).
constexpr uint8_t flags_version(uint8_t flags) { return static_cast<uint8_t>(flags >> 4); }
/// Whether a `flags` byte marks a response / event (rather than a request).
constexpr bool flags_is_reply(uint8_t flags) { return (flags & kFlagReply) != 0; }
/// Whether a `flags` byte marks a correlation id present in the header.
constexpr bool flags_has_correlation(uint8_t flags) { return (flags & kFlagCorrelation) != 0; }
/// Build a `flags` byte from a reply bit and a version (defaults to kVersion).
constexpr uint8_t make_flags(bool reply, uint8_t version = kVersion) {
  return static_cast<uint8_t>((version << 4) | (reply ? kFlagReply : 0));
}

/// Recommended standard values for the `type` field. A protocol may use these
/// generic transaction types (e.g. a register-style read/write interface) or
/// define its own type values within its module. `Custom` means the specific
/// operation is protocol-defined and (by convention) carried as the first
/// payload byte(s).
enum class Transaction : uint8_t {
  Write = 0x00,     ///< initiator -> responder write / command
  Read = 0x01,      ///< initiator -> responder read request
  WriteRead = 0x02, ///< combined write-then-read
  Custom = 0x03,    ///< protocol-defined; opcode carried in the payload
  // 0x04..0xFF are available for protocol-defined type values.
};

/// @brief Standard zlib CRC-32 (IEEE 802.3; poly 0xEDB88320 reflected, init
///        0xFFFFFFFF, final xor 0xFFFFFFFF).
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
struct Frame {
  uint8_t flags{make_flags(false)}; ///< version + reply bit + correlation bit
  uint8_t module{0};                ///< routing / protocol id
  uint8_t type{0};                  ///< message / transaction type within the module
  std::optional<uint16_t>
      correlation{};              ///< optional correlation / sequence id (see kFlagCorrelation)
  std::vector<uint8_t> payload{}; ///< payload bytes (may be empty)

  /// Whether this is a response / event rather than a request.
  bool is_reply() const { return flags_is_reply(flags); }
  /// The frame format version carried in `flags`.
  uint8_t version() const { return flags_version(flags); }
  /// Whether this frame carries a correlation id.
  bool has_correlation() const { return correlation.has_value(); }
  /// The `type` field as the standard Transaction enum (only meaningful when the
  /// module uses the standard values).
  Transaction transaction() const { return static_cast<Transaction>(type); }
};

/// @brief Build an encoded frame from an explicit `flags` byte.
/// @param flags Flags byte (see make_flags()); the correlation bit is set/cleared
///        to match @p correlation.
/// @param module Routing / protocol id.
/// @param type Message / transaction type within the module.
/// @param payload Payload bytes; must be <= kMaxPayloadSize.
/// @param correlation Optional u16 correlation / sequence id carried in the header.
/// @return The encoded frame bytes, or an empty vector if the payload is too large.
inline std::vector<uint8_t> build_frame(uint8_t flags, uint8_t module, uint8_t type,
                                        std::span<const uint8_t> payload = {},
                                        std::optional<uint16_t> correlation = std::nullopt) {
  if (payload.size() > kMaxPayloadSize)
    return {};
  // The correlation flag must agree with whether the field is emitted.
  flags = correlation ? static_cast<uint8_t>(flags | kFlagCorrelation)
                      : static_cast<uint8_t>(flags & ~kFlagCorrelation);
  std::vector<uint8_t> out;
  out.reserve(kMaxHeaderSize + payload.size() + kCrcSize);
  put_u16(out, kMagic);
  out.push_back(flags);
  out.push_back(module);
  out.push_back(type);
  if (correlation)
    put_u16(out, *correlation);
  put_u32(out, static_cast<uint32_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());
  put_u32(out, crc32(std::span<const uint8_t>(out.data(), out.size())));
  return out;
}

/// @brief Build an encoded frame, composing the `flags` byte from a reply bit.
inline std::vector<uint8_t> build_frame(bool reply, uint8_t module, uint8_t type,
                                        std::span<const uint8_t> payload = {},
                                        std::optional<uint16_t> correlation = std::nullopt) {
  return build_frame(make_flags(reply), module, type, payload, correlation);
}

/// @brief Incremental frame parser for the stream-frame protocol.
///
/// Feed arbitrary chunks of received bytes (USB bulk transfers, socket reads,
/// single bytes, ...) and it yields the complete, CRC-verified frames they
/// contain. On a bad magic, an oversized length field (> kMaxPayloadSize) or a
/// CRC mismatch it resynchronizes by discarding bytes until the next plausible
/// frame start, so a corrupted stream recovers at the next intact frame.
///
/// The parser yields EVERY CRC-verified frame; it does not filter by module or
/// type. Routing a multi-protocol stream to the right handler (and ignoring
/// unknown modules) is the job of espp::Dispatcher (or the caller).
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
      // Need at least magic + flags to know whether an optional field is present
      // (and therefore how big the header is).
      if (buffer_.size() - pos < 3)
        break;
      const auto header = std::span<const uint8_t>(buffer_).subspan(pos);
      const uint8_t flags = header[2];
      const size_t ext = flags_has_correlation(flags) ? kCorrelationSize : 0;
      const size_t header_size = kHeaderSize + ext; // len sits after the optional field(s)
      if (buffer_.size() - pos < header_size)
        break; // incomplete header; wait for more bytes
      const uint32_t len = get_u32(header.subspan(5 + ext));
      if (len > kMaxPayloadSize) {
        // Reject a remote-supplied length that exceeds the cap and resync one
        // byte past this (bogus) frame start.
        dropped_bytes_++;
        pos++;
        continue;
      }
      const size_t total = header_size + len + kCrcSize;
      if (buffer_.size() - pos < total)
        break; // incomplete frame; wait for more bytes
      const uint32_t expected = get_u32(header.subspan(header_size + len));
      const uint32_t actual = crc32(header.first(header_size + len));
      if (actual != expected) {
        // Corrupt frame; resync one byte past this frame start.
        dropped_bytes_++;
        pos++;
        continue;
      }
      Frame frame{};
      frame.flags = flags;
      frame.module = header[3];
      frame.type = header[4];
      if (ext)
        frame.correlation = get_u16(header.subspan(5));
      frame.payload.assign(header.begin() + header_size, header.begin() + header_size + len);
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
