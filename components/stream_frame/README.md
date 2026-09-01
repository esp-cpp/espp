# Stream Frame

A tiny, dependency-free wire-framing codec: CRC-32-verified, length-delimited,
typed frames plus an incremental, resynchronizing `StreamParser` for carrying
messages over any raw byte stream (USB vendor / CDC, TCP/UDP sockets, UART, ...).

It has **no** ESP-IDF / FreeRTOS dependency, so the framing builds and
unit-tests on a host with nothing more than a C++20 standard library. It is the
shared substrate under the OTA stream protocol (`espp::detail::ota_stream`),
the crash-dump service, and — via `espp::Dispatcher` — any number of
independent protocols multiplexed over one stream.

## Wire format

All multi-byte fields are little-endian:

```
[magic u16 = 0x4F54 ("OT")][type u8][len u32][payload: len bytes][crc32 u32]
```

- `magic` — `0x4F54` ("OT"); raw bytes `0x54` ('T') then `0x4F` ('O').
- `type` — an application message-type byte. Each protocol owns a disjoint range
  of the byte space; parsers ignore types they do not recognize, so multiple
  protocols coexist on one stream (see the `dispatcher` component).
- `len` — payload length, `<= kMaxPayloadSize` (4096). Oversized lengths are
  rejected and resynchronized past, bounding parser memory.
- `crc32` — standard zlib CRC-32 over `magic..payload`
  (`crc32("123456789") == 0xCBF43926`).

## API

- `std::vector<uint8_t> build_frame(uint8_t type, std::span<const uint8_t> payload = {})`
- `class StreamParser` — `feed(bytes) -> std::vector<Frame>`, `reset()`,
  `buffered()`, `dropped_bytes()`. `Frame` is `{ uint8_t type; std::vector<uint8_t> payload; }`.
- `crc32`, `put_u16`/`put_u32`, `get_u16`/`get_u32` helpers.

## Host tests

There is no ESP-IDF dependency, so `components/ota/test/ota_protocol_host_test.cpp`
exercises this codec directly on a host:

```
c++ -std=c++20 -Werror -I components/ota/include -I components/stream_frame/include \
    components/ota/test/ota_protocol_host_test.cpp -o test && ./test
```
