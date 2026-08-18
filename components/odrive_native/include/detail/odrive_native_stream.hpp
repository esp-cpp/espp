#pragma once

// ODrive legacy native (Fibre) UART *stream* framing.
//
// The packet codec lives in detail/odrive_native_core.hpp. Over USB, each bulk
// transfer carries exactly one packet and USB provides framing + reliability.
// Over a UART/serial link there is no such structure, so fibre's serial backend
// wraps every packet in a small stream frame with two CRCs:
//
//   [0xAA sync]
//   [len       u8   ]   packet length, MUST be < 128
//   [crc8      u8   ]   CRC8 over the two bytes [sync,len], init 0x42, poly 0x37
//   [packet    len  ]   the raw packet bytes (see odrive_native_core.hpp)
//   [crc16     u16 BE]  CRC16 over the packet bytes, init 0x1337, poly 0x3d65,
//                       transmitted big-endian (high byte first)
//
// Receiver validation trick (holds for these CRCs, and is what fibre relies on):
//   * CRC8  over [sync,len,crc8]           == 0
//   * CRC16 over [packet .. crc16 bytes]   == 0
//
// This header is host-buildable with nothing but a C++20 standard library and the
// core header (for odrive_crc16). It has zero ESP-IDF / FreeRTOS dependencies so
// the interop device shim and golden tests build with a plain `c++ -std=c++20`.

#include <cstdint>
#include <span>
#include <vector>

#include "detail/odrive_native_core.hpp"

namespace espp {
namespace detail {

/// The stream sync byte that begins every frame.
static constexpr uint8_t kStreamSync = 0xAA;
/// The stream framing caps a single packet at 127 bytes (len must be < 128).
static constexpr size_t kStreamMaxPacket = 127;

/// ODrive/fibre stream CRC8 (poly 0x37, init 0x42, non-reflected, MSB-first).
/// Fold a single byte through the running remainder.
inline uint8_t odrive_crc8_byte(uint8_t rem, uint8_t val) {
  rem ^= val;
  for (int i = 0; i < 8; i++)
    rem = (rem & 0x80) ? static_cast<uint8_t>((rem << 1) ^ 0x37) : static_cast<uint8_t>(rem << 1);
  return rem;
}

/// CRC8 over a buffer using the fibre stream init value (0x42).
inline uint8_t odrive_crc8(std::span<const uint8_t> data, uint8_t init = 0x42) {
  uint8_t r = init;
  for (uint8_t b : data)
    r = odrive_crc8_byte(r, b);
  return r;
}

/**
 * @brief Wrap one packet in a fibre serial stream frame.
 * @param packet The raw packet bytes (<= kStreamMaxPacket). The caller is
 *        responsible for ensuring the packet fits; e.g. the endpoint-0 JSON read
 *        response must be truncated so the packet is <= 127 bytes.
 * @return The framed byte stream: sync, len, crc8, packet, crc16(BE).
 */
inline std::vector<uint8_t> stream_frame(std::span<const uint8_t> packet) {
  std::vector<uint8_t> out;
  // The stream framing carries the packet length in a single byte that must be
  // < 128. A larger packet cannot be represented (len would wrap/truncate and
  // produce a malformed frame), so refuse it and return an empty vector.
  if (packet.size() > kStreamMaxPacket)
    return out;
  const uint8_t len = static_cast<uint8_t>(packet.size());
  out.reserve(packet.size() + 5);
  out.push_back(kStreamSync);
  out.push_back(len);
  const uint8_t header[2] = {kStreamSync, len};
  out.push_back(odrive_crc8(std::span<const uint8_t>(header, 2)));
  out.insert(out.end(), packet.begin(), packet.end());
  const uint16_t c = odrive_crc16(packet);
  out.push_back(static_cast<uint8_t>((c >> 8) & 0xff)); // big-endian: high byte
  out.push_back(static_cast<uint8_t>(c & 0xff));        //             low byte
  return out;
}

/**
 * @brief Stateful deframer for the fibre serial stream.
 *
 * Feed arbitrary chunks of received stream bytes with push(); it buffers partial
 * input, resynchronizes on the 0xAA sync byte, validates the CRC8 header and the
 * CRC16 trailer, and returns each complete, verified packet. A frame that fails
 * either CRC (or carries len >= 128) is discarded and the deframer resynchronizes
 * at the next 0xAA.
 */
class StreamDeframer {
public:
  /// Append received stream bytes and return any complete packets decoded.
  std::vector<std::vector<uint8_t>> push(std::span<const uint8_t> data) {
    buf_.insert(buf_.end(), data.begin(), data.end());
    std::vector<std::vector<uint8_t>> out;
    for (;;) {
      // Resync: drop everything before the first sync byte.
      size_t sync = 0;
      while (sync < buf_.size() && buf_[sync] != kStreamSync)
        ++sync;
      if (sync > 0)
        buf_.erase(buf_.begin(), buf_.begin() + sync);

      // Need at least the 3-byte header [sync,len,crc8].
      if (buf_.size() < 3)
        break;

      const uint8_t len = buf_[1];
      const uint8_t hcrc = buf_[2];
      const uint8_t header[2] = {buf_[0], len};
      if (len >= 128 || odrive_crc8(std::span<const uint8_t>(header, 2)) != hcrc) {
        // Bad header: drop the sync byte and hunt for the next one.
        buf_.erase(buf_.begin());
        continue;
      }

      // Need the full frame: header(3) + packet(len) + crc16(2).
      const size_t frame_len = 3 + static_cast<size_t>(len) + 2;
      if (buf_.size() < frame_len)
        break; // wait for more bytes

      std::span<const uint8_t> packet(buf_.data() + 3, len);
      const uint16_t got = static_cast<uint16_t>((static_cast<uint16_t>(buf_[3 + len]) << 8) |
                                                 buf_[3 + len + 1]); // big-endian
      if (odrive_crc16(packet) != got) {
        // Bad trailer CRC: drop the sync byte and resync.
        buf_.erase(buf_.begin());
        continue;
      }

      out.emplace_back(packet.begin(), packet.end());
      buf_.erase(buf_.begin(), buf_.begin() + frame_len);
    }
    return out;
  }

  /// Bytes currently buffered awaiting a complete frame (for diagnostics/tests).
  size_t buffered() const { return buf_.size(); }

private:
  std::vector<uint8_t> buf_;
};

} // namespace detail
} // namespace espp
