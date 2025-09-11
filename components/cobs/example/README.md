# COBS Example

This example demonstrates the COBS (Consistent Overhead Byte Stuffing) component with comprehensive tests and real-world usage scenarios.

## What This Example Tests

The example runs four main test suites:

1. **Single Packet Test**: Basic encoding/decoding of individual packets
2. **Streaming Encoder Test**: Batching multiple packets for transmission
3. **Streaming Decoder Test**: Processing fragmented incoming data streams
4. **Edge Cases Test**: Empty packets, all zeros, no zeros, and maximum block size

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
- Hex dumps of encoded/decoded data
- Verification results (PASS/FAIL)
- Buffer size information
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
[COBS Example/I][0.128]:
=== Streaming Encoder Test ===
[COBS Example/I][0.134]: Test 1: PASS - Multiple packets with zeros
[COBS Example/I][0.140]: Test 2: PASS - Empty encoder
[COBS Example/I][0.146]: Test 3: PASS - Single packet
[COBS Example/I][0.152]:
=== Streaming Decoder Test ===
[COBS Example/I][0.158]: Test 1: PASS - Multiple packets with fragmented input
[COBS Example/I][0.165]: Test 2: PASS - Empty decoder
[COBS Example/I][0.171]: Test 3: PASS - Single packet
[COBS Example/I][0.176]: Test 4: PASS - Incomplete packet
[COBS Example/I][0.182]:
=== Edge Cases Test ===
[COBS Example/I][0.188]: Test 1: PASS - Maximum block size (254 bytes)
[COBS Example/I][0.195]: Test 2: PASS - Alternating zeros and non-zeros
[COBS Example/I][0.202]: Test 3: PASS - Consecutive zeros
[COBS Example/I][0.208]: Large packet encoded: 1002 bytes
[COBS Example/I][0.214]: Test 4: PASS - Large packet (1000 bytes)
[COBS Example/I][0.220]: ============================
[COBS Example/I][0.226]: COBS example complete!
[COBS Example/I][0.231]: All tests completed successfully!
```

## Key Features Demonstrated

- **Efficient batching**: Multiple packets combined into transmission chunks
- **Fragmentation handling**: Incoming data processed in arbitrary chunks
- **Error handling**: Invalid packets are skipped gracefully
- **Edge case robustness**: Handles empty packets, all zeros, etc.

## See Also

- [COBS Component Documentation](../README.md) - Complete API reference
