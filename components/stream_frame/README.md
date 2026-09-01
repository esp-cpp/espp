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
[magic u16 = 0x4F54 ("OT")][flags u8][module u8][type u8]{[correlation u16] iff flags bit1}[len u32][payload][crc32 u32]
```

- `magic` — `0x4F54` ("OT"); raw bytes `0x54` ('T') then `0x4F` ('O').
- `flags` — **bit0 = reply**: the frame's direction in a request/response
  exchange — `0` = request (initiator→responder), `1` = response/event
  (responder→initiator). Which side is the "initiator" depends on the protocol's
  roles (a device answering a browser is a responder; a device that itself sends
  a request and awaits a response is an initiator). **bit1 = correlation
  present**; bits 2–3 reserved for future optional header fields; bits 4–7 =
  format version (currently 1).
- `module` — routing / protocol id (0..255). `espp::Dispatcher` routes on this.
- `type` — message / transaction type within the module (0..255). See the
  `Transaction` enum for recommended standard values (Write/Read/WriteRead/
  Custom); a protocol may otherwise define its own type values and carry a
  finer opcode in the payload.
- `correlation` — **optional** u16 (present only when `flags` bit1 is set): a
  protocol-defined correlation / sequence id, opaque to the codec, for matching
  a response to its request when more than one may be outstanding. It lives in
  the header (covered by the CRC), not the payload. Optional fields are
  flag-gated, so a frame without them is byte-identical to before and new
  optional fields can be added under bits 2–3 without another breaking change.
- `len` — payload length, `<= kMaxPayloadSize` (4096). Oversized lengths are
  rejected and resynchronized past, bounding parser memory.
- `crc32` — standard zlib CRC-32 over the whole header + payload
  (`crc32("123456789") == 0xCBF43926`).

`StreamParser::feed()` yields **every** CRC-verified frame — it does not filter
by module or type. Routing a multi-protocol stream to the right handler (and
ignoring unknown modules) is the job of `espp::Dispatcher` (or the caller).

## API

- `std::vector<uint8_t> build_frame(bool reply, uint8_t module, uint8_t type, std::span<const uint8_t> payload = {}, std::optional<uint16_t> correlation = std::nullopt)`
  (and a `flags`-byte overload). Pass `correlation` to include the optional id.
- `make_flags(bool reply, version = kVersion)`, `flags_is_reply()`,
  `flags_has_correlation()`, `flags_version()`.
- `enum class Transaction { Write, Read, WriteRead, Custom }`.
- `class StreamParser` — `feed(bytes) -> std::vector<Frame>`, `reset()`,
  `buffered()`, `dropped_bytes()`. `Frame` is
  `{ uint8_t flags; uint8_t module; uint8_t type; std::optional<uint16_t> correlation; std::vector<uint8_t> payload; }`
  with `is_reply()` / `has_correlation()` / `version()` / `transaction()` accessors.
- `crc32`, `put_u16`/`put_u32`, `get_u16`/`get_u32` helpers.

## Host tests

```
c++ -std=c++20 -Werror -I components/stream_frame/include \
    components/stream_frame/test/stream_frame_host_test.cpp -o test && ./test
```
