#pragma once

#include "cobs.hpp"
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <vector>

namespace espp {

/**
 * @brief Streaming decoder for multiple COBS-encoded packets
 *
 * Useful for processing incoming data streams where packets may arrive
 * in fragments or multiple packets may arrive together.
 */
class CobsStreamDecoder {
public:
  CobsStreamDecoder() = default;

  /**
   * @brief Add encoded data to the decoder buffer
   *
   * @param data New encoded data span
   */
  void add_data(std::span<const uint8_t> data);

  /**
   * @brief Add encoded data to the decoder buffer (move semantics)
   *
   * @param data New encoded data vector (will be moved)
   */
  void add_data(std::vector<uint8_t> &&data);

  /**
   * @brief Try to extract the next complete packet. Removes the extracted data from the buffer.
   *
   * @return Decoded packet data, or empty if no complete packet found
   */
  std::optional<std::vector<uint8_t>> extract_packet();

  /**
   * @brief Access remaining unprocessed data for debug purposes
   *
   * @return Const reference to buffered data that hasn't been processed yet
   */
  const std::vector<uint8_t> &remaining_data() const;

  /**
   * @brief Get the size of buffered data
   *
   * @return Number of bytes currently buffered
   */
  size_t buffer_size() const;

  /**
   * @brief Clear all buffered data
   */
  void clear();

private:
  std::vector<uint8_t> buffer_;
  mutable std::mutex mutex_;
};

/**
 * @brief Streaming encoder for multiple packets
 *
 * Useful for batching multiple packets together for transmission
 * or for building up data to send in chunks.
 */
class CobsStreamEncoder {
public:
  CobsStreamEncoder() = default;

  /**
   * @brief Add a packet to be encoded
   *
   * @param data Packet data span
   */
  void add_packet(std::span<const uint8_t> data);

  /**
   * @brief Add a packet to be encoded (move semantics)
   *
   * @param data Packet data vector (will be moved)
   */
  void add_packet(std::vector<uint8_t> &&data);

  /**
   * @brief Get all encoded data as a single buffer for debug purposes
   *
   * @return All encoded packets concatenated, const reference
   */
  const std::vector<uint8_t> &get_encoded_data() const;

  /**
   * @brief Extract encoded data up to a maximum size
   *
   * @param max_size Maximum number of bytes to extract
   * @return Encoded data up to max_size bytes
   */
  std::vector<uint8_t> extract_data(size_t max_size);

  /**
   * @brief Extract encoded data directly to a buffer
   *
   * @param output Output buffer to write data to
   * @param max_size Maximum number of bytes to extract
   * @return Number of bytes actually written to output
   */
  size_t extract_data(uint8_t *output, size_t max_size);

  /**
   * @brief Get the current buffer size
   *
   * @return Number of bytes currently buffered
   */
  size_t buffer_size() const;

  /**
   * @brief Clear all buffered data
   */
  void clear();

private:
  std::vector<uint8_t> buffer_;
  mutable std::mutex mutex_;
};

} // namespace espp
