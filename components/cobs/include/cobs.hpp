#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace espp {

/**
 * @brief COBS (Consistent Overhead Byte Stuffing) encoder/decoder
 *
 * Provides single-packet encoding and decoding using the COBS algorithm
 * with 0 as the delimiter.
 * COBS encoding can add at most ⌈n/254⌉ + 1 bytes overhead. Plus 1 byte for the delimiter
 * COBS changes the size of the packet by at least 1 byte, so it's not possible to encode in
 * place. MAX_BLOCK_SIZE = 254 is the maximum number of non-zero bytes in an encoded block.
 *
 * @see https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */
class Cobs {
public:
  /**
   * @brief Encode a single packet
   *
   * @param data Input data to encode
   * @return Encoded data with COBS encoding and delimiter
   */
  static std::vector<uint8_t> encode_packet(std::span<const uint8_t> data);

  /**
   * @brief Encode a single packet to existing buffer
   *
   * @param data Input data to encode
   * @param output Output buffer (must be large enough)
   * @return Number of bytes written to output
   */
  static size_t encode_packet(std::span<const uint8_t> data, uint8_t *output);

  /**
   * @brief Decode a single packet from COBS-encoded data
   *
   * @param encoded_data COBS-encoded data
   * @return Decoded packet data, or empty if invalid
   */
  static std::vector<uint8_t> decode_packet(std::span<const uint8_t> encoded_data);

  /**
   * @brief Decode a single packet to existing buffer
   *
   * @param encoded_data COBS-encoded data
   * @param output Output buffer (must be large enough)
   * @return Number of bytes written to output, or 0 if decoding failed
   */
  static size_t decode_packet(std::span<const uint8_t> encoded_data, uint8_t *output);

private:
  static constexpr uint8_t DELIMITER = 0x00;
  static constexpr size_t MAX_BLOCK_SIZE = 254;
};

} // namespace espp
