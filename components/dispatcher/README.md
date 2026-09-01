# Dispatcher

`espp::Dispatcher` multiplexes several independent framed protocols over a
single byte stream. It parses the `stream_frame` codec **once** and routes each
complete frame to a per-module handler by the frame's `module` byte — so OTA,
crash-dump inspection, a CAN bridge and an application's own control channel can
share one USB vendor / CDC / socket / UART link without interfering.
Header-only and dependency-free (only `stream_frame` + the standard library),
so it also builds and unit-tests on a host.

Rather than run a separate `StreamParser` per protocol over the same bytes (each
re-buffering the whole stream and needing its own reset-on-overflow
bookkeeping), the Dispatcher owns the one parser and dispatches by module id.

## Module id

The `module` byte (0..255) is the routing key — a full byte, so up to 256
protocols can coexist on one stream. The message/transaction `type` and the
request/reply direction (`flags`) travel with the frame and are handed to the
module's handler untouched; the Dispatcher does not interpret them. espp
built-in protocols use, for example:

| Module | Protocol   |
|--------|------------|
| 0      | OTA        |
| 4      | crash dump |
| 5      | CAN bridge |

A device-side dispatcher registers the modules it serves; frames for an
unregistered module (including the device's own replies echoed back, which carry
the reply flag) are silently ignored. Application code may assign any unused
module id to its own protocol — nothing is hard-wired to a specific service.

## API

- `void register_module(uint8_t module_id, handler_fn handler)` /
  `void unregister_module(uint8_t module_id)` / `bool has_module(uint8_t)`
  where `handler_fn = std::function<void(const stream_frame::Frame&)>`.
- `void feed(std::span<const uint8_t> data)` — parse + route.
- `void dispatch(const stream_frame::Frame&)` — route an already-parsed frame.
- `void reset()` — drop buffered bytes (reconnect / RX overflow).
- `buffered()`, `dropped_bytes()`.

```cpp
espp::Dispatcher dispatcher;
dispatcher.register_module(0, [&](const espp::stream_frame::Frame &f) {
  // handle OTA frames: f.type, f.is_reply(), f.payload
});
dispatcher.register_module(4, [&](const espp::stream_frame::Frame &f) {
  // handle crash-dump frames
});
usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { dispatcher.feed(data); });
```

## Host tests

```
c++ -std=c++20 -Werror -I components/dispatcher/include -I components/stream_frame/include \
    components/dispatcher/test/dispatcher_host_test.cpp -o test && ./test
```
