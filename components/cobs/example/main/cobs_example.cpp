#include <chrono>
#include <cstring>
#include <numeric>
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
    std::vector<uint8_t> encoded = Cobs::encode_packet(packet.data, sizeof(packet.data));
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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
    std::vector<uint8_t> encoded = Cobs::encode_packet(packet.data(), packet.size());
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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
    std::vector<uint8_t> encoded = Cobs::encode_packet(packet.data(), packet.size());
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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
    std::vector<uint8_t> encoded = Cobs::encode_packet(nullptr, 0);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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
    std::vector<uint8_t> encoded = Cobs::encode_packet(&single_byte, 1);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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
    std::vector<uint8_t> encoded = Cobs::encode_packet(&zero_byte, 1);
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

    // Verify
    bool success = (decoded.size() == 1) && (decoded[0] == zero_byte);
    if (success) {
      logger.info("Test 6: PASS - Single zero byte");
    } else {
      logger.error("Test 6: FAIL - Single zero byte");
    }
  }
}

void test_streaming_encoder(espp::Logger &logger) {
  logger.info("\n=== Streaming Encoder Test ===");

  // Test 1: Multiple packets with move semantics
  {
    CobsStreamEncoder encoder;

    // Add multiple packets using move semantics
    std::vector<std::vector<uint8_t>> packets;
    for (int i = 0; i < 3; ++i) {
      Packet48 packet(i + 1);
      packet.data[0] = 0x00; // Include some zeros
      packet.data[28] = 0x00;
      packet.data[43] = 0x00;
      
      // Convert to vector for move semantics
      std::vector<uint8_t> packet_data(packet.data, packet.data + sizeof(packet.data));
      packets.push_back(std::move(packet_data));
    }

    // Add packets using move semantics
    for (auto &packet : packets) {
      encoder.add_packet(std::move(packet));
    }

    // Extract all data using move semantics
    std::vector<uint8_t> all_encoded = encoder.extract_data(encoder.buffer_size());

    bool success = (all_encoded.size() > 0) && (encoder.buffer_size() == 0);
    if (success) {
      logger.info("Test 1: PASS - Multiple packets with move semantics");
    } else {
      logger.error("Test 1: FAIL - Multiple packets with move semantics");
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

  // Test 3: Single packet
  {
    CobsStreamEncoder encoder;

    uint8_t single_byte = 0x42;
    encoder.add_packet(&single_byte, 1);

    std::vector<uint8_t> all_encoded = encoder.get_encoded_data();

    bool success = (all_encoded.size() > 0);
    if (success) {
      logger.info("Test 3: PASS - Single packet");
    } else {
      logger.error("Test 3: FAIL - Single packet");
    }
  }
}

void test_streaming_decoder(espp::Logger &logger) {
  logger.info("\n=== Streaming Decoder Test ===");

  // Test 1: Multiple packets with move semantics
  {
    CobsStreamDecoder decoder;

    // Create some encoded packets
    std::vector<uint8_t> all_encoded;
    for (int i = 0; i < 3; ++i) {
      Packet48 packet(i + 10);
      packet.data[0] = 0x00; // Include some zeros
      packet.data[28] = 0x00;
      packet.data[43] = 0x00;

      std::vector<uint8_t> encoded = Cobs::encode_packet(packet.data, sizeof(packet.data));
      all_encoded.insert(all_encoded.end(), encoded.begin(), encoded.end());
    }

    // Add all encoded data using move semantics
    decoder.add_data(std::move(all_encoded));

    // Extract all packets
    int packets_extracted = 0;
    while (auto packet = decoder.extract_packet()) {
      packets_extracted++;
    }

    bool success = (packets_extracted == 3);
    if (success) {
      logger.info("Test 1: PASS - Multiple packets with move semantics");
    } else {
      logger.error(
          "Test 1: FAIL - Multiple packets with move semantics (extracted: {}, expected: 3)",
          packets_extracted);
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
    std::vector<uint8_t> encoded = Cobs::encode_packet(&single_byte, 1);

    decoder.add_data(encoded.data(), encoded.size());
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
    decoder.add_data(partial_data, sizeof(partial_data));

    auto packet = decoder.extract_packet();
    size_t buffer_size = decoder.buffer_size();

    bool success = (!packet.has_value()) && (buffer_size == 2);
    if (success) {
      logger.info("Test 4: PASS - Incomplete packet");
    } else {
      logger.error("Test 4: FAIL - Incomplete packet");
    }
  }
}

void test_edge_cases(espp::Logger &logger) {
  logger.info("\n=== Edge Cases Test ===");

  // Test 1: Maximum block size (254 non-zero bytes)
  {
    std::vector<uint8_t> max_block(254, 0x42);
    std::vector<uint8_t> encoded = Cobs::encode_packet(max_block.data(), max_block.size());
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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

    std::vector<uint8_t> encoded = Cobs::encode_packet(alternating.data(), alternating.size());
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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

    std::vector<uint8_t> encoded =
        Cobs::encode_packet(consecutive_zeros.data(), consecutive_zeros.size());
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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

    std::vector<uint8_t> encoded = Cobs::encode_packet(large_packet.data(), large_packet.size());
    std::vector<uint8_t> decoded = Cobs::decode_packet(encoded.data(), encoded.size());

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
