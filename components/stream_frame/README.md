# Stream Frame

A tiny, dependency-free wire-framing codec (v2): CRC-32-verified,
length-delimited frames carrying a routing `module` id, a message/transaction
`type`, and a `flags` byte (version + request/reply direction), plus an
incremental, resynchronizing `StreamParser` — for carrying messages over any
raw byte stream (USB vendor / CDC, TCP/UDP sockets, UART, ...).

It has **no** ESP-IDF / FreeRTOS dependency, so the framing builds and
unit-tests on a host with nothing more than a C++20 standard library. It is the
shared substrate under the OTA stream protocol (`espp::detail::ota_stream`),
the crash-dump service, the CAN bridge, and — via `espp::Dispatcher` — any
number of independent protocols multiplexed over one stream.

## Wire format (v2)

All multi-byte fields are little-endian:

```
[magic u16 = 0x4F54 ("OT")][flags u8][module u8][type u8][len u32][payload: len bytes][crc32 u32]
```

- `magic` — `0x4F54` ("OT"); raw bytes `0x54` ('T') then `0x4F` ('O').
- `flags` — bit0 = reply (0 = request host→device, 1 = reply/event
  device→host); bits 1–3 reserved; bits 4–7 = format version (currently 1).
- `module` — routing / protocol id (0..255). `espp::Dispatcher` routes on this.
- `type` — message / transaction type within the module (0..255). See the
  `Transaction` enum for recommended standard values (Write/Read/WriteRead/
  Custom); a protocol may otherwise define its own type values and carry a
  finer opcode in the payload.
- `len` — payload length, `<= kMaxPayloadSize` (4096). Oversized lengths are
  rejected and resynchronized past, bounding parser memory.
- `crc32` — standard zlib CRC-32 over `magic..payload`
  (`crc32("123456789") == 0xCBF43926`).

`StreamParser::feed()` yields **every** CRC-verified frame — it does not filter
by module or type. Routing a multi-protocol stream to the right handler (and
ignoring unknown modules) is the job of `espp::Dispatcher` (or the caller).

## API

- `std::vector<uint8_t> build_frame(uint8_t flags, uint8_t module, uint8_t type, std::span<const uint8_t> payload = {})`
  and a `build_frame(bool reply, module, type, payload)` convenience overload.
- `make_flags(bool reply, version = kVersion)`, `flags_is_reply()`, `flags_version()`.
- `enum class Transaction { Write, Read, WriteRead, Custom }`.
- `class StreamParser` — `feed(bytes) -> std::vector<Frame>`, `reset()`,
  `buffered()`, `dropped_bytes()`. `Frame` is
  `{ uint8_t flags; uint8_t module; uint8_t type; std::vector<uint8_t> payload; }`
  with `is_reply()` / `version()` / `transaction()` accessors.
- `crc32`, `put_u16`/`put_u32`, `get_u16`/`get_u32` helpers.

## Host tests

```
c++ -std=c++20 -Werror -I components/stream_frame/include \
    components/stream_frame/test/stream_frame_host_test.cpp -o test && ./test
```
