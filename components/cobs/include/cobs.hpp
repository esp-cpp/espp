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
   * @brief Calculate maximum encoded size for a given payload length
   *
   * @param payload_len Length of input data
   * @return Maximum number of bytes needed for encoding (including delimiter)
   */
  static constexpr size_t max_encoded_size(size_t payload_len) {
    return payload_len + (payload_len / 254) + 2;
  }

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
   * @param output Output buffer span (must be at least max_encoded_size)
   * @return Number of bytes written to output
   */
  static size_t encode_packet(std::span<const uint8_t> data, std::span<uint8_t> output);

  /**
   * @brief Calculate maximum decoded size for a given encoded length
   *
   * @param encoded_len Length of COBS-encoded data
   * @return Maximum number of bytes needed for decoding (accounts for delimiter)
   */
  static constexpr size_t max_decoded_size(size_t encoded_len) {
    if (encoded_len == 0)
      return 0;
    // For decoding, the maximum decoded size is the encoded size minus the delimiter
    // (worst case: no zeros in original data, so minimal COBS overhead)
    return encoded_len - 1;
  }

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
   * @param output Output buffer span (must be at least max_decoded_size)
   * @return Number of bytes written to output, or 0 if decoding failed
   */
  static size_t decode_packet(std::span<const uint8_t> encoded_data, std::span<uint8_t> output);

private:
  static constexpr uint8_t DELIMITER = 0x00;
  static constexpr size_t MAX_BLOCK_SIZE = 254;
};

} // namespace espp
