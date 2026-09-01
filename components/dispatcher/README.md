# Dispatcher

`espp::Dispatcher` multiplexes several independent framed protocols over a
single byte stream. It parses the `stream_frame` codec **once** and routes each
complete frame to a per-module handler by the message-type byte's high-nibble
module id — so OTA, crash-dump inspection, a CAN bridge and an application's own
control channel can share one USB vendor / CDC / socket / UART link without
interfering. Header-only and dependency-free (only `stream_frame` + the standard
library), so it also builds and unit-tests on a host.

Rather than run a separate `StreamParser` per protocol over the same bytes (each
re-buffering the whole stream and needing its own reset-on-overflow
bookkeeping), the Dispatcher owns the one parser and dispatches by module id.

## Module id convention

The module id is the high nibble of the message-type byte (`type >> 4`). espp
built-in protocols place their request opcodes so each protocol occupies one
high nibble, and use bit 7 to mark device→host replies:

| Module id | Protocol   | Requests | Replies         |
|-----------|------------|----------|-----------------|
| 0         | OTA        | `0x0X`   | `0x8X`          |
| 4         | crash dump | `0x4X`   | `0xCX`          |
| 5         | CAN bridge | `0x5X`   | `0xDX` (example)|

A device-side dispatcher registers the request modules (0, 4, 5, ...); it never
receives reply-typed frames (high nibble 8..15), and if one arrives it lands on
an unregistered module and is ignored, so requests and replies of one protocol
can never be confused. Application code may assign any unused module id to its
own protocol. Frames for an unregistered module are silently ignored.

## API

- `void register_module(uint8_t module_id, handler_fn handler)` /
  `void unregister_module(uint8_t module_id)` / `bool has_module(uint8_t)`
  where `handler_fn = std::function<void(uint8_t type, std::span<const uint8_t> payload)>`.
- `void feed(std::span<const uint8_t> data)` — parse + route.
- `void dispatch(const stream_frame::Frame&)` — route an already-parsed frame.
- `void reset()` — drop buffered bytes (reconnect / RX overflow).
- `static constexpr uint8_t module_of(uint8_t type)`, `buffered()`, `dropped_bytes()`.

## Host tests

```
c++ -std=c++20 -Werror -I components/dispatcher/include -I components/stream_frame/include \
    components/dispatcher/test/dispatcher_host_test.cpp -o test && ./test
```
