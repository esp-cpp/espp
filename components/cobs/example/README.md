# COBS Example

This example demonstrates the COBS (Consistent Overhead Byte Stuffing) component with comprehensive tests and real-world usage scenarios.

## What This Example Tests

The example runs four main test suites:

1. **Single Packet Test**: Basic encoding/decoding of individual packets with span-based API
2. **Streaming Encoder Test**: Batching multiple packets for transmission with buffer management
3. **Streaming Decoder Test**: Processing fragmented incoming data streams with debugging support
4. **Edge Cases Test**: Empty packets, all zeros, no zeros, and maximum block size

## Detailed Test Coverage

### Single Packet Test (Tests 1-12)
- **Tests 1-6**: Basic encoding/decoding with various data patterns (zeros at different positions, no zeros, all zeros, empty packets, single bytes)
- **Tests 7-8**: Buffer-based encoding/decoding using `std::span` API for embedded systems
- **Tests 9-10**: Buffer size validation - ensures functions return 0 when buffers are too small
- **Tests 11-12**: Static API usage - demonstrates `max_encoded_size()` and `max_decoded_size()` for buffer allocation

### Streaming Encoder Test (Tests 1-5)
- **Test 1**: Multiple packets with move semantics for efficient memory usage
- **Test 2**: Empty encoder behavior
- **Test 3**: Buffer-based `extract_data()` for direct buffer output
- **Test 4**: Single packet encoding
- **Test 5**: `clear()` method for buffer management

### Streaming Decoder Test (Tests 1-6)
- **Test 1**: Multiple packets with move semantics for efficient processing
- **Test 2**: Empty decoder behavior
- **Test 3**: Single packet decoding
- **Test 4**: Incomplete packet handling (fragmented data)
- **Test 5**: `remaining_data()` access for debugging and buffer inspection
- **Test 6**: `clear()` method for buffer management

### Edge Cases Test (Tests 1-4)
- **Test 1**: Maximum block size (254 non-zero bytes) - tests COBS overhead limits
- **Test 2**: Alternating zeros and non-zeros - tests complex encoding patterns
- **Test 3**: Consecutive zeros - tests zero handling edge cases
- **Test 4**: Large packet (1000 bytes) - tests performance with realistic packet sizes

## Test Scenario: 48-byte Packets

The example uses 48-byte packets to simulate a realistic communication protocol:
- **Packet size**: 48 bytes (includes zeros to test COBS encoding)
- **Chunk size**: 100 bytes for encoder, 80 bytes for decoder (simulates bus limitations)
- **Multiple packets**: Tests batching and fragmentation handling

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

## Monitor Commands

- **Exit monitor**: Type `Ctrl-]`
- **Reset board**: Type `Ctrl+R`

## Expected Output

The example will output:
- Verification results (PASS/FAIL) for all 27 test cases
- Buffer size information and validation results
- Static API usage demonstrations with size calculations
- Buffer-based encoding/decoding test results
- Streaming encoder/decoder functionality tests
- Edge case handling verification
- Summary of all test results

```
I (305) main_task: Calling app_main()
[COBS Example/I][0.042]: Starting COBS example!
[COBS Example/I][0.042]: COBS Encoder/Decoder Example
[COBS Example/I][0.048]: ============================
[COBS Example/I][0.053]:
=== Single Packet Test ===
[COBS Example/I][0.059]: Test 1: Packet with zeros at positions 0, 28, 43
[COBS Example/I][0.066]: Test 1: PASS - Zeros at various positions
[COBS Example/I][0.073]: Test 2: Packet with no zeros (1-48)
[COBS Example/I][0.079]: Test 2: PASS - No zeros
[COBS Example/I][0.084]: Test 3: Packet with all zeros
[COBS Example/I][0.090]: Test 3: PASS - All zeros
[COBS Example/I][0.095]: Test 4: Empty packet
[COBS Example/I][0.100]: Test 4: PASS - Empty packet
[COBS Example/I][0.105]: Test 5: Single byte packet (0x42)
[COBS Example/I][0.111]: Test 5: PASS - Single byte
[COBS Example/I][0.117]: Test 6: Single zero byte
[COBS Example/I][0.122]: Test 6: PASS - Single zero byte
[COBS Example/I][0.128]: Test 7: Buffer-based encoding
[COBS Example/I][0.133]: Test 7: PASS - Buffer-based encoding
[COBS Example/I][0.140]: Test 8: Buffer-based decoding
[COBS Example/I][0.145]: Test 8: PASS - Buffer-based decoding
[COBS Example/I][0.151]: Test 9: Buffer size validation - encoding with too small buffer
[COBS Example/I][0.160]: Test 9: PASS - Buffer size validation (encoding)
[COBS Example/I][0.167]: Test 10: Buffer size validation - decoding with too small buffer
[COBS Example/I][0.176]: Test 10: PASS - Buffer size validation (decoding)
[COBS Example/I][0.183]: Test 11: Static API usage - max_encoded_size
[COBS Example/I][0.190]: Test 11: PASS - Static API max_encoded_size (required: 9, written: 9)
[COBS Example/I][0.199]: Test 12: Static API usage - max_decoded_size
[COBS Example/I][0.207]: Test 12: PASS - Static API max_decoded_size (required: 8, written: 7, original: 7)
[COBS Example/I][0.217]:
=== Streaming Encoder Test ===
[COBS Example/I][0.223]: Test 1: PASS - Multiple packets with move semantics
[COBS Example/I][0.230]: Test 2: PASS - Empty encoder
[COBS Example/I][0.236]: Test 3: PASS - Buffer-based extract_data
[COBS Example/I][0.242]: Test 3: PASS - Single packet
[COBS Example/I][0.248]: Test 5: clear() method
[COBS Example/I][0.253]: Test 5: PASS - clear() method
[COBS Example/I][0.259]:
=== Streaming Decoder Test ===
[COBS Example/I][0.265]: Test 1: PASS - Multiple packets with move semantics
[COBS Example/I][0.272]: Test 2: PASS - Empty decoder
[COBS Example/I][0.278]: Test 3: PASS - Single packet
[COBS Example/I][0.283]: Test 4: PASS - Incomplete packet
[COBS Example/I][0.289]: Test 5: remaining_data() access
[COBS Example/I][0.295]: Test 5: PASS - remaining_data() access
[COBS Example/I][0.301]: Test 6: clear() method
[COBS Example/I][0.306]: Test 6: PASS - clear() method
[COBS Example/I][0.312]:
=== Edge Cases Test ===
[COBS Example/I][0.317]: Test 1: PASS - Maximum block size (254 bytes)
[COBS Example/I][0.324]: Test 2: PASS - Alternating zeros and non-zeros
[COBS Example/I][0.331]: Test 3: PASS - Consecutive zeros
[COBS Example/I][0.338]: Test 4: PASS - Large packet (1000 bytes)
[COBS Example/I][0.344]: ============================
[COBS Example/I][0.349]: COBS example complete!
[COBS Example/I][0.354]: All tests completed successfully!
```

## Key Features Demonstrated

- **Span-based API**: Zero-copy operations using `std::span` for efficient data handling
- **Buffer-based encoding/decoding**: Direct buffer operations for embedded systems with limited memory
- **Buffer size calculation**: Static API methods for compile-time buffer size estimation
- **Buffer validation**: Comprehensive buffer size checking and overflow protection
- **Efficient batching**: Multiple packets combined into transmission chunks with move semantics
- **Fragmentation handling**: Incoming data processed in arbitrary chunks with debugging support
- **Error handling**: Invalid packets are skipped gracefully with proper error reporting
- **Edge case robustness**: Handles empty packets, all zeros, large packets, and complex patterns
- **Thread-safe streaming**: Streaming classes with internal locking for multi-threaded environments
- **Memory management**: RAII-based memory management with clear() methods for buffer control

## See Also

- [COBS Component Documentation](../README.md) - Complete API reference
