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

## Key Features Demonstrated

- **Efficient batching**: Multiple packets combined into transmission chunks
- **Fragmentation handling**: Incoming data processed in arbitrary chunks
- **Error handling**: Invalid packets are skipped gracefully
- **Edge case robustness**: Handles empty packets, all zeros, etc.

## See Also

- [COBS Component Documentation](../README.md) - Complete API reference
