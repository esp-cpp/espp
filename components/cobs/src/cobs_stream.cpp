#include "cobs_stream.hpp"
#include <algorithm>

namespace espp {

void CobsStreamDecoder::add_data(std::span<const uint8_t> data) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Append data directly to buffer (zero-copy for contiguous data)
  buffer_.insert(buffer_.end(), data.begin(), data.end());
}

void CobsStreamDecoder::add_data(std::vector<uint8_t> &&data) {
  if (data.empty())
    return;

  std::lock_guard<std::mutex> lock(mutex_);
  // Append new data to buffer using move semantics
  size_t old_size = buffer_.size();
  buffer_.resize(old_size + data.size());
  std::move(data.begin(), data.end(), buffer_.begin() + old_size);
}

std::optional<std::vector<uint8_t>> CobsStreamDecoder::extract_packet() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty()) {
    return std::nullopt;
  }

  // Find the first delimiter
  auto delimiter_it = std::find(buffer_.begin(), buffer_.end(), 0x00);
  if (delimiter_it == buffer_.end()) {
    return std::nullopt; // No complete packet found
  }

  // Calculate packet boundaries
  size_t packet_end = std::distance(buffer_.begin(), delimiter_it);
  size_t bytes_consumed = packet_end + 1; // Include delimiter

  if (packet_end == 0) {
    // Empty packet, just consume the delimiter
    buffer_.erase(buffer_.begin(), buffer_.begin() + bytes_consumed);
    return std::vector<uint8_t>{};
  }

  // Decode the packet (include the delimiter in the data passed to decode_packet)
  std::vector<uint8_t> decoded = Cobs::decode_packet(std::span{buffer_.data(), packet_end + 1});

  // Remove consumed data from buffer
  buffer_.erase(buffer_.begin(), buffer_.begin() + bytes_consumed);

  if (decoded.empty()) {
    // Invalid packet
    return std::nullopt;
  }

  return decoded;
}

const std::vector<uint8_t> &CobsStreamDecoder::remaining_data() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_;
}

size_t CobsStreamDecoder::buffer_size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size();
}

void CobsStreamDecoder::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.clear();
}

void CobsStreamEncoder::add_packet(std::span<const uint8_t> data) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (data.empty())
    return;

  // Calculate worst-case encoded size using the static API
  size_t max_encoded_size = Cobs::max_encoded_size(data.size());

  // Resize buffer to accommodate the encoded data
  size_t old_size = buffer_.size();
  buffer_.resize(old_size + max_encoded_size);

  // Encode directly to the buffer
  size_t encoded_size =
      Cobs::encode_packet(data, std::span{buffer_.data() + old_size, max_encoded_size});

  // Resize buffer to actual encoded size
  buffer_.resize(old_size + encoded_size);
}

void CobsStreamEncoder::add_packet(std::vector<uint8_t> &&data) {
  if (data.empty())
    return;

  std::lock_guard<std::mutex> lock(mutex_);

  // Calculate worst-case encoded size using the static API
  size_t max_encoded_size = Cobs::max_encoded_size(data.size());

  // Resize buffer to accommodate the encoded data
  size_t old_size = buffer_.size();
  buffer_.resize(old_size + max_encoded_size);

  // Encode directly to the buffer
  size_t encoded_size =
      Cobs::encode_packet(data, std::span{buffer_.data() + old_size, max_encoded_size});

  // Resize buffer to actual encoded size
  buffer_.resize(old_size + encoded_size);
}

std::vector<uint8_t> CobsStreamEncoder::extract_data(size_t max_size) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty()) {
    return {};
  }

  size_t extract_size = std::min(max_size, buffer_.size());

  // If extracting all data, use move semantics for efficiency
  if (extract_size == buffer_.size()) {
    std::vector<uint8_t> result = std::move(buffer_);
    buffer_.clear(); // Ensure buffer is in a valid state
    return result;
  }

  // For partial extraction, we still need to copy (can't move part of a vector)
  std::vector<uint8_t> result(buffer_.begin(), buffer_.begin() + extract_size);
  buffer_.erase(buffer_.begin(), buffer_.begin() + extract_size);
  return result;
}

size_t CobsStreamEncoder::extract_data(uint8_t *output, size_t max_size) {
  if (output == nullptr) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (buffer_.empty()) {
    return 0;
  }

  size_t extract_size = std::min(max_size, buffer_.size());

  // Copy data directly to output buffer
  std::copy(buffer_.begin(), buffer_.begin() + extract_size, output);

  // Remove extracted data from buffer
  buffer_.erase(buffer_.begin(), buffer_.begin() + extract_size);

  return extract_size;
}

size_t CobsStreamEncoder::buffer_size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.size();
}

void CobsStreamEncoder::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.clear();
}

const std::vector<uint8_t> &CobsStreamEncoder::get_encoded_data() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_;
}

} // namespace espp
