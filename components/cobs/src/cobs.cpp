#include "cobs.hpp"
#include <cstring>

namespace espp {

std::vector<uint8_t> Cobs::encode_packet(std::span<const uint8_t> data) {
  if (data.empty())
    return {};

  // Calculate maximum encoded size using the static API
  size_t max_encoded_size = Cobs::max_encoded_size(data.size());
  std::vector<uint8_t> encoded(max_encoded_size);

  size_t encoded_size = encode_packet(data, std::span{encoded});
  encoded.resize(encoded_size);

  return encoded;
}

size_t Cobs::encode_packet(std::span<const uint8_t> data, std::span<uint8_t> output) {
  if (output.empty() || data.empty())
    return 0;

  // Calculate required buffer size using the static API
  size_t max_encoded_size = Cobs::max_encoded_size(data.size());

  // Check if output buffer is large enough
  if (output.size() < max_encoded_size)
    return 0;

  uint8_t *encode =
      output.data(); // Encoded byte pointer points to first value data will be copied to
  uint8_t *codep =
      encode++; // Output code pointer (points to first value of the block (where the code goes))
  uint8_t code = 1; // Code value

  for (const uint8_t byte : data) {
    if (byte) { // Byte not zero, write it
      *encode++ = byte;
      ++code;
    }

    if (!byte || code == 0xff) { // Input is zero or block completed, restart
      *codep = code;
      code = 1;
      codep = encode;
      if (!byte)
        ++encode;
    }
  }
  *codep = code; // Write final code value

  // Add delimiter at the end
  *encode++ = DELIMITER;

  return static_cast<size_t>(encode - output.data());
}

std::vector<uint8_t> Cobs::decode_packet(std::span<const uint8_t> encoded_data) {
  if (encoded_data.empty())
    return {};

  // Find the delimiter to determine packet boundary
  const uint8_t *delimiter = static_cast<const uint8_t *>(
      std::memchr(encoded_data.data(), DELIMITER, encoded_data.size()));
  if (!delimiter) {
    return {}; // No delimiter found
  }

  size_t packet_length = static_cast<size_t>(delimiter - encoded_data.data());

  // Decode the packet
  std::vector<uint8_t> decoded(packet_length); // Worst case: same size as encoded
  size_t decoded_length = decode_packet(encoded_data.subspan(0, packet_length), std::span{decoded});

  if (decoded_length == 0) {
    return {}; // Decoding failed
  }

  decoded.resize(decoded_length);
  return decoded;
}

size_t Cobs::decode_packet(std::span<const uint8_t> encoded_data, std::span<uint8_t> output) {
  if (output.empty() || encoded_data.empty())
    return 0;

  // Calculate maximum decoded size using the static API
  size_t max_decoded_size = Cobs::max_decoded_size(encoded_data.size());

  // Check if output buffer is large enough
  if (output.size() < max_decoded_size)
    return 0;

  const uint8_t *byte = encoded_data.data(); // Encoded input byte pointer
  uint8_t *decode = output.data();           // Decoded output byte pointer
  const uint8_t *end = encoded_data.data() + encoded_data.size();

  uint8_t code = 0xff; // Initialize code outside loop to check final state
  for (uint8_t block = 0; byte < end; block--) {
    if (block) { // Decode block byte
      *decode++ = *byte++;
      if (decode >= output.data() + output.size()) // Buffer overflow
        return 0;
    } else {
      if (byte >= end) // Unexpected end of data
        return 0;
      block = *byte++;
      if (block && (code != 0xff)) { // Encoded zero, write it when transitioning between blocks
        *decode++ = 0;
        if (decode >= output.data() + output.size()) // Buffer overflow
          return 0;
      }
      code = block;
      if (!code) { // Delimiter code found
        break;
      }
    }
  }

  // Check if we reached the end of data (for vector version) or found delimiter (for raw version)
  if (code != 0 && byte >= end) {
    // We've processed all data and the last code is not 0, which means we didn't find a delimiter
    // This is expected when called from the vector version that strips the delimiter
    return static_cast<size_t>(decode - output.data());
  } else if (code != 0) {
    return 0; // No delimiter found - incomplete packet
  }

  return static_cast<size_t>(decode - output.data());
}

} // namespace espp
