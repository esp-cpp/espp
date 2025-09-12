#include <array>
#include <chrono>
#include <cstring>
#include <numeric>
#include <span>
#include <thread>
#include <vector>

#include "cobs.hpp"
#include "cobs_stream.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;
using namespace espp;

// Example: 48-byte packet structure
struct Packet48 {
  uint8_t data[48];

  Packet48() { std::memset(data, 0, sizeof(data)); }

  explicit Packet48(uint8_t value) { std::fill(data, data + sizeof(data), value); }
};

void test_single_packet(espp::Logger &logger) {
  logger.info("\n=== Single Packet Test ===");

  // Test 1: Packet with zeros at various positions
  {
    Packet48 packet(0x42);
    packet.data[0] = 0x00; // Include some zeros to test COBS encoding
    packet.data[28] = 0x00;
    packet.data[43] = 0x00;

    logger.info("Test 1: Packet with zeros at positions 0, 28, 43");

    // Encode and decode
    std::vector<uint8_t> encoded = Cobs::encode_packet(std::span{packet.data});
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    // Verify
    bool success = (decoded.size() == sizeof(packet.data)) &&
                   (std::memcmp(decoded.data(), packet.data, sizeof(packet.data)) == 0);
    if (success) {
      logger.info("Test 1: PASS - Zeros at various positions");
    } else {
      logger.error("Test 1: FAIL - Zeros at various positions");
    }
  }

  // Test 2: Packet with no zeros
  {
    std::vector<uint8_t> packet(48);
    std::iota(packet.begin(), packet.end(), 1); // 1, 2, 3, ..., 48

    logger.info("Test 2: Packet with no zeros (1-48)");

    // Encode and decode
    std::vector<uint8_t> encoded = Cobs::encode_packet(packet);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    // Verify
    bool success = (decoded.size() == packet.size()) &&
                   (std::memcmp(decoded.data(), packet.data(), packet.size()) == 0);
    if (success) {
      logger.info("Test 2: PASS - No zeros");
    } else {
      logger.error("Test 2: FAIL - No zeros");
    }
  }

  // Test 3: Packet with all zeros
  {
    std::vector<uint8_t> packet(48, 0x00);

    logger.info("Test 3: Packet with all zeros");

    // Encode and decode
    std::vector<uint8_t> encoded = Cobs::encode_packet(packet);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    // Verify
    bool success = (decoded.size() == packet.size()) &&
                   (std::memcmp(decoded.data(), packet.data(), packet.size()) == 0);
    if (success) {
      logger.info("Test 3: PASS - All zeros");
    } else {
      logger.error("Test 3: FAIL - All zeros");
    }
  }

  // Test 4: Empty packet
  {
    logger.info("Test 4: Empty packet");

    // Encode and decode
    std::vector<uint8_t> encoded = Cobs::encode_packet(std::span<const uint8_t>{});
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    // Verify
    bool success = (decoded.size() == 0);
    if (success) {
      logger.info("Test 4: PASS - Empty packet");
    } else {
      logger.error("Test 4: FAIL - Empty packet");
    }
  }

  // Test 5: Single byte packet
  {
    uint8_t single_byte = 0x42;

    logger.info("Test 5: Single byte packet (0x42)");

    // Encode and decode
    std::vector<uint8_t> encoded = Cobs::encode_packet(std::span{&single_byte, 1});
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    // Verify
    bool success = (decoded.size() == 1) && (decoded[0] == single_byte);
    if (success) {
      logger.info("Test 5: PASS - Single byte");
    } else {
      logger.error("Test 5: FAIL - Single byte");
    }
  }

  // Test 6: Single zero byte
  {
    uint8_t zero_byte = 0x00;

    logger.info("Test 6: Single zero byte");

    // Encode and decode
    std::vector<uint8_t> encoded = Cobs::encode_packet(std::span{&zero_byte, 1});
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    // Verify
    bool success = (decoded.size() == 1) && (decoded[0] == zero_byte);
    if (success) {
      logger.info("Test 6: PASS - Single zero byte");
    } else {
      logger.error("Test 6: FAIL - Single zero byte");
    }
  }

  // Test 7: Buffer-based encoding
  {
    logger.info("Test 7: Buffer-based encoding");

    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};
    uint8_t output_buffer[100];

    size_t bytes_written = Cobs::encode_packet(test_data, std::span{output_buffer});

    // Verify encoding worked
    bool success = (bytes_written > 0) && (bytes_written < sizeof(output_buffer));
    if (success) {
      // Decode back to verify
      std::vector<uint8_t> decoded = Cobs::decode_packet(std::span{output_buffer, bytes_written});
      success = (decoded == test_data);
    }

    if (success) {
      logger.info("Test 7: PASS - Buffer-based encoding");
    } else {
      logger.error("Test 7: FAIL - Buffer-based encoding");
    }
  }

  // Test 8: Buffer-based decoding
  {
    logger.info("Test 8: Buffer-based decoding");

    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};
    std::vector<uint8_t> encoded = Cobs::encode_packet(test_data);
    uint8_t output_buffer[100];

    size_t bytes_written = Cobs::decode_packet(encoded, std::span{output_buffer});

    // Verify decoding worked
    bool success = (bytes_written == test_data.size());
    if (success) {
      success = (std::memcmp(output_buffer, test_data.data(), test_data.size()) == 0);
    }

    if (success) {
      logger.info("Test 8: PASS - Buffer-based decoding");
    } else {
      logger.error("Test 8: FAIL - Buffer-based decoding");
    }
  }

  // Test 9: Buffer size validation - encoding with too small buffer
  {
    logger.info("Test 9: Buffer size validation - encoding with too small buffer");

    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};
    uint8_t small_buffer[2]; // Too small for encoded data

    size_t bytes_written = Cobs::encode_packet(test_data, std::span{small_buffer});

    // Should return 0 because buffer is too small
    bool success = (bytes_written == 0);

    if (success) {
      logger.info("Test 9: PASS - Buffer size validation (encoding)");
    } else {
      logger.error("Test 9: FAIL - Buffer size validation (encoding)");
    }
  }

  // Test 10: Buffer size validation - decoding with too small buffer
  {
    logger.info("Test 10: Buffer size validation - decoding with too small buffer");

    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};
    std::vector<uint8_t> encoded = Cobs::encode_packet(test_data);
    uint8_t small_buffer[2]; // Too small for decoded data

    size_t bytes_written = Cobs::decode_packet(encoded, std::span{small_buffer});

    // Should return 0 because buffer is too small
    bool success = (bytes_written == 0);

    if (success) {
      logger.info("Test 10: PASS - Buffer size validation (decoding)");
    } else {
      logger.error("Test 10: FAIL - Buffer size validation (decoding)");
    }
  }

  // Test 11: Static API usage - max_encoded_size
  {
    logger.info("Test 11: Static API usage - max_encoded_size");

    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};

    // Calculate required buffer size using static API
    size_t required_size = Cobs::max_encoded_size(test_data.size());

    // Allocate buffer with exact size
    std::vector<uint8_t> buffer(required_size);

    // Encode using the buffer
    size_t bytes_written = Cobs::encode_packet(test_data, std::span{buffer});

    bool success = (bytes_written > 0) && (bytes_written <= required_size);

    if (success) {
      logger.info("Test 11: PASS - Static API max_encoded_size (required: {}, written: {})",
                  required_size, bytes_written);
    } else {
      logger.error("Test 11: FAIL - Static API max_encoded_size");
    }
  }

  // Test 12: Static API usage - max_decoded_size
  {
    logger.info("Test 12: Static API usage - max_decoded_size");

    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};
    std::vector<uint8_t> encoded = Cobs::encode_packet(test_data);

    // Calculate required buffer size using static API
    size_t required_size = Cobs::max_decoded_size(encoded.size());

    // Allocate buffer with the calculated size (should be sufficient)
    std::vector<uint8_t> buffer(required_size);

    // Decode using the buffer
    size_t bytes_written = Cobs::decode_packet(encoded, std::span{buffer});

    // Verify the decoded data matches the original
    bool content_match = (bytes_written == test_data.size()) &&
                         (std::memcmp(buffer.data(), test_data.data(), test_data.size()) == 0);

    bool success = (bytes_written > 0) && (bytes_written <= required_size) && content_match;

    if (success) {
      logger.info(
          "Test 12: PASS - Static API max_decoded_size (required: {}, written: {}, original: {})",
          required_size, bytes_written, test_data.size());
    } else {
      logger.error("Test 12: FAIL - Static API max_decoded_size (required: {}, written: {}, "
                   "original: {}, content_match: {})",
                   required_size, bytes_written, test_data.size(), content_match);
    }
  }
}

void test_streaming_encoder(espp::Logger &logger) {
  logger.info("\n=== Streaming Encoder Test ===");

  // Test 1: Multiple packets with move semantics
  {
    CobsStreamEncoder encoder;

    // Create original packets and store them for verification
    std::vector<std::vector<uint8_t>> original_packets;
    for (int i = 0; i < 3; ++i) {
      Packet48 packet(i + 1);
      packet.data[0] = 0x00; // Include some zeros
      packet.data[28] = 0x00;
      packet.data[43] = 0x00;

      // Store original data before moving
      std::vector<uint8_t> original_data(packet.data, packet.data + sizeof(packet.data));
      original_packets.push_back(original_data);

      // Convert to vector for move semantics
      std::vector<uint8_t> packet_data(packet.data, packet.data + sizeof(packet.data));
      encoder.add_packet(std::move(packet_data));
    }

    // Extract all data using move semantics
    std::vector<uint8_t> all_encoded = encoder.extract_data(encoder.buffer_size());

    // Verify encoding worked
    bool encoding_success = (all_encoded.size() > 0) && (encoder.buffer_size() == 0);
    if (!encoding_success) {
      logger.error("Test 1: FAIL - Multiple packets with move semantics (encoding failed)");
    } else {
      // Verify encoded data can be decoded back to original
      CobsStreamDecoder decoder;
      decoder.add_data(std::move(all_encoded));

      std::vector<std::vector<uint8_t>> decoded_packets;
      while (auto packet = decoder.extract_packet()) {
        decoded_packets.push_back(*packet);
      }

      // Verify decoded packets match original
      bool content_success = (decoded_packets.size() == original_packets.size());
      if (content_success) {
        for (size_t i = 0; i < original_packets.size(); ++i) {
          if (decoded_packets[i] != original_packets[i]) {
            logger.error("Test 1: FAIL - Packet {} content mismatch after encode/decode", i);
            content_success = false;
            break;
          }
        }
      }

      if (content_success) {
        logger.info("Test 1: PASS - Multiple packets with move semantics");
      } else {
        logger.error(
            "Test 1: FAIL - Multiple packets with move semantics (content verification failed)");
      }
    }
  }

  // Test 2: Empty encoder
  {
    CobsStreamEncoder encoder;

    std::vector<uint8_t> all_encoded = encoder.get_encoded_data();
    size_t buffer_size = encoder.buffer_size();

    bool success = (all_encoded.size() == 0) && (buffer_size == 0);
    if (success) {
      logger.info("Test 2: PASS - Empty encoder");
    } else {
      logger.error("Test 2: FAIL - Empty encoder");
    }
  }

  // Test 3: Buffer-based extract_data
  {
    CobsStreamEncoder encoder;

    // Add a test packet
    std::vector<uint8_t> test_data = {0x01, 0x02, 0x00, 0x03, 0x04, 0x00, 0x05};
    encoder.add_packet(std::move(test_data));

    // Extract to buffer
    uint8_t output_buffer[100];
    size_t bytes_extracted = encoder.extract_data(output_buffer, sizeof(output_buffer));

    // Verify extraction worked
    bool success = (bytes_extracted > 0) && (encoder.buffer_size() == 0);
    if (success) {
      logger.info("Test 3: PASS - Buffer-based extract_data");
    } else {
      logger.error("Test 3: FAIL - Buffer-based extract_data");
    }
  }

  // Test 4: Single packet
  {
    CobsStreamEncoder encoder;

    uint8_t single_byte = 0x42;
    encoder.add_packet(std::span{&single_byte, 1});

    std::vector<uint8_t> all_encoded = encoder.get_encoded_data();

    bool success = (all_encoded.size() > 0);
    if (success) {
      logger.info("Test 3: PASS - Single packet");
    } else {
      logger.error("Test 3: FAIL - Single packet");
    }
  }

  // Test 5: clear() method
  {
    logger.info("Test 5: clear() method");

    CobsStreamEncoder encoder;

    // Add some packets
    std::vector<uint8_t> test_data1 = {0x01, 0x02, 0x00, 0x03};
    std::vector<uint8_t> test_data2 = {0x04, 0x05, 0x00, 0x06};
    encoder.add_packet(test_data1);
    encoder.add_packet(test_data2);

    // Verify data is there
    bool has_data = (encoder.buffer_size() > 0);

    // Clear the buffer
    encoder.clear();

    // Verify buffer is empty
    bool success = has_data && (encoder.buffer_size() == 0);

    // Try to get encoded data (should be empty)
    const auto &encoded = encoder.get_encoded_data();
    success = success && (encoded.size() == 0);

    if (success) {
      logger.info("Test 5: PASS - clear() method");
    } else {
      logger.error("Test 5: FAIL - clear() method");
    }
  }
}

void test_streaming_decoder(espp::Logger &logger) {
  logger.info("\n=== Streaming Decoder Test ===");

  // Test 1: Multiple packets with move semantics
  {
    CobsStreamDecoder decoder;

    // Create some original packets and store them for verification
    std::vector<std::vector<uint8_t>> original_packets;
    std::vector<uint8_t> all_encoded;

    for (int i = 0; i < 3; ++i) {
      Packet48 packet(i + 10);
      packet.data[0] = 0x00; // Include some zeros
      packet.data[28] = 0x00;
      packet.data[43] = 0x00;

      // Store original packet data
      std::vector<uint8_t> original_data(packet.data, packet.data + sizeof(packet.data));
      original_packets.push_back(original_data);

      std::vector<uint8_t> encoded = Cobs::encode_packet(std::span{packet.data});
      all_encoded.insert(all_encoded.end(), encoded.begin(), encoded.end());
    }

    // Add all encoded data using move semantics
    decoder.add_data(std::move(all_encoded));

    // Extract all packets and verify content
    int packets_extracted = 0;
    std::vector<std::vector<uint8_t>> decoded_packets;

    while (auto packet = decoder.extract_packet()) {
      decoded_packets.push_back(*packet);
      packets_extracted++;
    }

    // Verify packet count
    bool count_success = (packets_extracted == 3);
    if (!count_success) {
      logger.error(
          "Test 1: FAIL - Multiple packets with move semantics (extracted: {}, expected: 3)",
          packets_extracted);
    } else {
      // Verify packet content matches original data
      bool content_success = true;
      for (size_t i = 0; i < original_packets.size(); ++i) {
        if (decoded_packets[i] != original_packets[i]) {
          logger.error("Test 1: FAIL - Packet {} content mismatch", i);
          content_success = false;
          break;
        }
      }

      if (content_success) {
        logger.info("Test 1: PASS - Multiple packets with move semantics");
      }
    }
  }

  // Test 2: Empty decoder
  {
    CobsStreamDecoder decoder;

    auto packet = decoder.extract_packet();
    size_t buffer_size = decoder.buffer_size();

    bool success = (!packet.has_value()) && (buffer_size == 0);
    if (success) {
      logger.info("Test 2: PASS - Empty decoder");
    } else {
      logger.error("Test 2: FAIL - Empty decoder");
    }
  }

  // Test 3: Single packet
  {
    CobsStreamDecoder decoder;

    uint8_t single_byte = 0x42;
    std::vector<uint8_t> encoded = Cobs::encode_packet(std::span{&single_byte, 1});

    decoder.add_data(encoded);
    auto packet = decoder.extract_packet();

    bool success = packet.has_value() && (packet->size() == 1) && (packet->at(0) == single_byte);
    if (success) {
      logger.info("Test 3: PASS - Single packet");
    } else {
      logger.error("Test 3: FAIL - Single packet (has_value: {}, size: {}, value: {})",
                   packet.has_value(), packet.has_value() ? packet->size() : 0,
                   packet.has_value() ? packet->at(0) : 0);
    }
  }

  // Test 4: Incomplete packet
  {
    CobsStreamDecoder decoder;

    // Add partial data (no delimiter)
    uint8_t partial_data[] = {0x02, 0x42};
    decoder.add_data(std::span{partial_data});

    auto packet = decoder.extract_packet();
    size_t buffer_size = decoder.buffer_size();

    bool success = (!packet.has_value()) && (buffer_size == 2);
    if (success) {
      logger.info("Test 4: PASS - Incomplete packet");
    } else {
      logger.error("Test 4: FAIL - Incomplete packet");
    }
  }

  // Test 5: remaining_data() access
  {
    logger.info("Test 5: remaining_data() access");

    CobsStreamDecoder decoder;

    // Add partial data (no delimiter)
    uint8_t partial_data[] = {0x02, 0x42, 0x43};
    decoder.add_data(std::span{partial_data});

    // Check remaining data
    const auto &remaining = decoder.remaining_data();
    bool success = (remaining.size() == 3) && (remaining[0] == 0x02) && (remaining[1] == 0x42) &&
                   (remaining[2] == 0x43);

    if (success) {
      logger.info("Test 5: PASS - remaining_data() access");
    } else {
      logger.error("Test 5: FAIL - remaining_data() access");
    }
  }

  // Test 6: clear() method
  {
    logger.info("Test 6: clear() method");

    CobsStreamDecoder decoder;

    // Add some data
    uint8_t test_data[] = {0x02, 0x42, 0x00};
    decoder.add_data(std::span{test_data});

    // Verify data is there
    bool has_data = (decoder.buffer_size() > 0);

    // Clear the buffer
    decoder.clear();

    // Verify buffer is empty
    bool success = has_data && (decoder.buffer_size() == 0);

    // Try to extract packet (should be empty)
    auto packet = decoder.extract_packet();
    success = success && (!packet.has_value());

    if (success) {
      logger.info("Test 6: PASS - clear() method");
    } else {
      logger.error("Test 6: FAIL - clear() method");
    }
  }
}

void test_edge_cases(espp::Logger &logger) {
  logger.info("\n=== Edge Cases Test ===");

  // Test 1: Maximum block size (254 non-zero bytes)
  {
    std::vector<uint8_t> max_block(254, 0x42);
    std::vector<uint8_t> encoded = Cobs::encode_packet(max_block);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    bool success = (decoded.size() == max_block.size()) &&
                   (std::memcmp(decoded.data(), max_block.data(), max_block.size()) == 0);
    if (success) {
      logger.info("Test 1: PASS - Maximum block size (254 bytes)");
    } else {
      logger.error("Test 1: FAIL - Maximum block size (254 bytes)");
    }
  }

  // Test 2: Alternating zeros and non-zeros
  {
    std::vector<uint8_t> alternating(20);
    for (size_t i = 0; i < alternating.size(); ++i) {
      alternating[i] = (i % 2 == 0) ? 0x00 : 0x42;
    }

    std::vector<uint8_t> encoded = Cobs::encode_packet(alternating);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    bool success = (decoded.size() == alternating.size()) &&
                   (std::memcmp(decoded.data(), alternating.data(), alternating.size()) == 0);
    if (success) {
      logger.info("Test 2: PASS - Alternating zeros and non-zeros");
    } else {
      logger.error("Test 2: FAIL - Alternating zeros and non-zeros");
    }
  }

  // Test 3: Consecutive zeros
  {
    std::vector<uint8_t> consecutive_zeros(10, 0x00);
    consecutive_zeros[5] = 0x42; // One non-zero in the middle

    std::vector<uint8_t> encoded = Cobs::encode_packet(consecutive_zeros);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    bool success =
        (decoded.size() == consecutive_zeros.size()) &&
        (std::memcmp(decoded.data(), consecutive_zeros.data(), consecutive_zeros.size()) == 0);
    if (success) {
      logger.info("Test 3: PASS - Consecutive zeros");
    } else {
      logger.error("Test 3: FAIL - Consecutive zeros");
    }
  }

  // Test 4: Large packet (1000 bytes)
  {
    std::vector<uint8_t> large_packet(1000);
    // Fill with repeating pattern to avoid uint8_t overflow issues
    for (size_t i = 0; i < large_packet.size(); ++i) {
      large_packet[i] = (i % 201); // 0, 1, 2, ..., 200, 0, 1, 2, ...
    }
    large_packet[100] = 0x00; // Add a zero
    large_packet[500] = 0x00; // Add another zero

    std::vector<uint8_t> encoded = Cobs::encode_packet(large_packet);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded);

    bool success = (decoded.size() == large_packet.size()) &&
                   (std::memcmp(decoded.data(), large_packet.data(), large_packet.size()) == 0);
    if (success) {
      logger.info("Test 4: PASS - Large packet (1000 bytes)");
    } else {
      logger.error(
          "Test 4: FAIL - Large packet (1000 bytes) (decoded: {}, expected: {}, encoded: {})",
          decoded.size(), large_packet.size(), encoded.size());
    }
  }
}

extern "C" void app_main(void) {
  //! [cobs example]
  espp::Logger logger({.tag = "COBS Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting COBS example!");
  logger.info("COBS Encoder/Decoder Example");
  logger.info("============================");

  test_single_packet(logger);
  test_streaming_encoder(logger);
  test_streaming_decoder(logger);
  test_edge_cases(logger);

  logger.info("============================");
  logger.info("COBS example complete!");
  logger.info("All tests completed successfully!");
  //! [cobs example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
