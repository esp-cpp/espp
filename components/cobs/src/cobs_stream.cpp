#include "cobs_stream.hpp"
#include <algorithm>

namespace espp {

void CobsStreamDecoder::add_data(const uint8_t* data, size_t length) {
    if(data == nullptr) 
        return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    // Append new data to buffer
    size_t old_size = buffer_.size();
    buffer_.resize(old_size + length);
    std::copy(data, data + length, buffer_.begin() + old_size);
}

std::optional<std::vector<uint8_t>> CobsStreamDecoder::extract_packet() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (buffer_.empty()) {
        return std::nullopt;
    }
    
    // Find the first delimiter
    auto delimiter_it = std::find(buffer_.begin(), buffer_.end(), 0x00);
    if (delimiter_it == buffer_.end()) {
        return std::nullopt;  // No complete packet found
    }
    
    // Calculate packet boundaries
    size_t packet_end = std::distance(buffer_.begin(), delimiter_it);
    size_t bytes_consumed = packet_end + 1;  // Include delimiter
    
    if (packet_end == 0) {
        // Empty packet, just consume the delimiter
        buffer_.erase(buffer_.begin(), buffer_.begin() + bytes_consumed);
        return std::vector<uint8_t>{};
    }
    
    // Decode the packet (include the delimiter in the data passed to decode_packet)
    std::vector<uint8_t> decoded = Cobs::decode_packet(buffer_.data(), packet_end + 1);
    if (decoded.empty()) {
        // Invalid packet - let caller decide what to do
        return std::nullopt;
    }
    
    // Remove consumed data from buffer
    buffer_.erase(buffer_.begin(), buffer_.begin() + bytes_consumed);
    
    return decoded;
}

const std::vector<uint8_t>& CobsStreamDecoder::remaining_data() const {
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

void CobsStreamEncoder::add_packet(const uint8_t* data, size_t length) {
    if(data == nullptr) 
        return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    // Encode the packet
    std::vector<uint8_t> encoded = Cobs::encode_packet(data, length);
    
    // Append to buffer
    size_t old_size = buffer_.size();
    buffer_.resize(old_size + encoded.size());
    std::copy(encoded.begin(), encoded.end(), buffer_.begin() + old_size);
}

std::vector<uint8_t> CobsStreamEncoder::extract_data(size_t max_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (buffer_.empty()) {
        return {};
    }
    
    size_t extract_size = std::min(max_size, buffer_.size());
    std::vector<uint8_t> result(buffer_.begin(), buffer_.begin() + extract_size);
    
    // Remove extracted data from buffer
    buffer_.erase(buffer_.begin(), buffer_.begin() + extract_size);
    
    return result;
}

size_t CobsStreamEncoder::buffer_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
}

void CobsStreamEncoder::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
}

} // namespace espp
