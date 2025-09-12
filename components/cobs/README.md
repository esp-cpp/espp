# COBS (Consistent Overhead Byte Stuffing) Component

[![Badge](https://components.espressif.com/components/espp/cobs/badge.svg)](https://components.espressif.com/components/espp/cobs)

The COBS component provides efficient encoding and decoding for the Consistent Overhead Byte Stuffing algorithm. COBS is a packet framing protocol that ensures reliable packet boundaries in communication streams by eliminating zero bytes from the data and using them as packet delimiters.

## Features

- **Zero-byte elimination**: Removes all zero bytes from data, using them as packet delimiters
- **Consistent overhead**: Maximum overhead of ⌈n/254⌉ + 1 bytes per packet
- **Simple framing**: Uses zero bytes as reliable packet delimiters
- **Streaming support**: Handle fragmented data streams and batch multiple packets
- **Thread-safe**: Streaming classes are thread-safe with internal locking
- **Memory efficient**: Minimal overhead with RAII memory management
- **Real-time friendly**: O(n) complexity with predictable performance

## Design Differences from Other COBS Libraries

This ESPP COBS implementation has some design choices that may differ from other COBS libraries:

### **Delimiter Handling**
- **ESPP COBS**: Automatically adds a `0x00` delimiter at the end of encoded packets
- **Other libraries**: May not add delimiters, requiring manual framing
- **Impact**: ESPP encoded data includes framing, making it ready for transmission

### **Empty Packet Handling**
- **ESPP COBS**: Ignores empty packets (length = 0) for performance optimization
- **Other libraries**: May encode empty packets as a single `0x01` byte
- **Impact**: Empty packets are filtered out during encoding, reducing overhead

### **Compatibility Notes**
When integrating with other COBS libraries:
- **Decoding**: ESPP can decode data from other libraries by adding the missing `0x00` delimiter
- **Encoding**: Other libraries may need to add `0x00` delimiters to decode ESPP-encoded data
- **Empty packets**: Handle empty packets separately if compatibility is required

### **Example: Design Differences**

```cpp
// Input data with zeros
std::vector<uint8_t> data = {0x01, 0x02, 0x00, 0x03, 0x04};

// ESPP COBS encoding (includes delimiter)
std::vector<uint8_t> espp_encoded = Cobs::encode_packet(std::span{data});
// Result: {0x03, 0x01, 0x02, 0x03, 0x03, 0x04, 0x00}

// Other COBS libraries (no delimiter)
// Result: {0x03, 0x01, 0x02, 0x03, 0x03, 0x04}

// Empty packet handling
std::vector<uint8_t> empty_data = {};
espp_encoded = Cobs::encode_packet(std::span{empty_data});
// ESPP Result: {} (ignored)
// Other libraries: {0x01} (encoded as single byte)
```

## API Design

The COBS implementation provides two levels of API:

### 1. Single Packet API (`cobs.hpp`)
- **`Cobs::encode_packet()`** - Encode a single packet with COBS
- **`Cobs::decode_packet()`** - Decode a single COBS-encoded packet
- **`Cobs::max_encoded_size()`** - Calculate required buffer size for encoding
- **`Cobs::max_decoded_size()`** - Calculate required buffer size for decoding
- Simple, stateless functions for basic encoding/decoding with buffer size calculation

### 2. Streaming API (`cobs_stream.hpp`)
- **`CobsStreamEncoder`** - Batch multiple packets for transmission
- **`CobsStreamDecoder`** - Handle fragmented incoming data streams
- Stateful classes for complex communication scenarios

## Usage

### Basic Single Packet Encoding/Decoding

```cpp
#include "cobs.hpp"
#include <span>

// Encode a packet (returns vector)
std::vector<uint8_t> data = {0x01, 0x02, 0x00, 0x03, 0x04};
std::vector<uint8_t> encoded = Cobs::encode_packet(std::span{data});

// Decode a packet (returns vector)
std::vector<uint8_t> decoded = Cobs::decode_packet(std::span{encoded});

// Buffer-based encoding (for embedded systems with limited memory)
uint8_t input_data[] = {0x01, 0x02, 0x00, 0x03, 0x04};
size_t required_size = Cobs::max_encoded_size(sizeof(input_data));
uint8_t output_buffer[100];  // Must be >= required_size

size_t bytes_written = Cobs::encode_packet(std::span{input_data}, std::span{output_buffer});
if (bytes_written > 0) {
    // Successfully encoded, output_buffer contains encoded data
}

// Buffer-based decoding
size_t decoded_size = Cobs::max_decoded_size(encoded.size());
uint8_t decode_buffer[100];  // Must be >= decoded_size

size_t bytes_decoded = Cobs::decode_packet(std::span{encoded}, std::span{decode_buffer});
if (bytes_decoded > 0) {
    // Successfully decoded, decode_buffer contains original data
}
```

### Streaming Encoder for Batching

```cpp
#include "cobs_stream.hpp"
#include <span>

// Create streaming encoder
CobsStreamEncoder encoder;

// Add multiple packets using std::span (zero-copy for contiguous data)
for (auto& packet : packets) {
    encoder.add_packet(std::span{packet});  // Zero-copy from vector
}

// Or with arrays (zero-copy)
uint8_t data[] = {0x01, 0x02, 0x00, 0x03};
encoder.add_packet(std::span{data});

// Extract data in chunks suitable for transmission
while (encoder.buffer_size() > 0) {
    auto chunk = encoder.extract_data(max_chunk_size);
    send_over_bus(chunk);
}

// Or extract directly to a buffer
uint8_t tx_buffer[256];
size_t bytes_extracted = encoder.extract_data(tx_buffer, sizeof(tx_buffer));
if (bytes_extracted > 0) {
    send_over_bus(std::span{tx_buffer, bytes_extracted});
}
```

### Streaming Decoder for Fragmented Data

```cpp
#include "cobs_stream.hpp"
#include <span>

// Create streaming decoder
CobsStreamDecoder decoder;

// Add received data (may be fragmented) using std::span
decoder.add_data(std::span{received_data, received_length});

// Or with vectors (zero-copy)
std::vector<uint8_t> received_vec = get_received_data();
decoder.add_data(std::span{received_vec});  // Zero-copy from vector

// Extract complete packets
while (auto packet = decoder.extract_packet()) {
    process_packet(*packet);
}

// Check remaining data for debugging
if (decoder.buffer_size() > 0) {
    const auto& remaining = decoder.remaining_data();
    // Handle incomplete data or debug buffer contents
}

// Clear buffer if needed
decoder.clear();
```


**Buffer Size Formulas:**
- **`max_encoded_size(payload_len)`**: `payload_len + ⌈payload_len/254⌉ + 2`
- **`max_decoded_size(encoded_len)`**: `encoded_len - 1` (accounts for delimiter)

## COBS Algorithm Details

COBS works by:
1. **Eliminating zeros**: All zero bytes are removed from the data
2. **Adding overhead bytes**: Each block of non-zero data gets a length prefix
3. **Using zero as delimiter**: A single zero byte marks the end of each packet
4. **Consistent overhead**: Maximum overhead is ⌈n/254⌉ + 1 bytes per packet

### Example Encoding
```
Input:  [0x01, 0x02, 0x00, 0x03, 0x04]
Output: [0x03, 0x01, 0x02, 0x02, 0x03, 0x04, 0x00]
        ^     ^     ^     ^     ^     ^     ^
        |     |     |     |     |     |     delimiter
        |     |     |     |     |     data
        |     |     |     |     data
        |     |     |     length (2 bytes follow)
        |     |     data
        |     data
        length (3 bytes follow)
```

## Performance Characteristics

- **Encoding overhead**: At most ⌈n/254⌉ + 1 bytes per packet
- **Decoding complexity**: O(n) where n is packet size
- **Speed**: Optimized for embedded systems with minimal overhead
- **Large packet support**: Tested and verified with packets up to 1000+ bytes
- **Memory efficiency**: Uses RAII for automatic memory management
- **Zero-copy API**: `std::span` interface eliminates unnecessary data copies
- **Direct buffer encoding**: Packets are encoded directly into internal buffers
- **Buffer size calculation**: Static methods provide compile-time buffer size estimation
- **Embedded-friendly**: Buffer-based APIs for systems with limited heap allocation

## Thread Safety

The streaming classes (`CobsStreamEncoder` and `CobsStreamDecoder`) are **thread-safe** with internal locking:

- **Internal mutexes**: All buffer operations are protected by mutexes
- **Atomic operations**: Each method call is atomic and thread-safe
- **No external synchronization needed**: Safe to use from multiple threads

**Note**: The single packet API (`Cobs::encode_packet` and `Cobs::decode_packet`) is stateless and inherently thread-safe.

## Example

The [example](./example) demonstrates comprehensive COBS usage including:
- Single packet encoding and decoding with various data patterns using `std::span` API
- Buffer-based encoding/decoding for embedded systems with limited memory
- Static buffer size calculation API usage
- Streaming encoder for batching multiple packets with zero-copy efficiency
- Streaming decoder for processing fragmented data with buffer management
- Edge cases and error handling (empty packets, large packets, alternating patterns)
- Buffer size validation and overflow protection
- Comprehensive test suite with test cases covering all functionality
- Performance characteristics for various packet sizes (up to 1000+ bytes)

## References

- [COBS Wikipedia Article](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing)
- [COBS C Implementation](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing#Implementation)
- [HDLC Framing Comparison](https://en.wikipedia.org/wiki/High-Level_Data_Link_Control)