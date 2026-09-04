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

| Module    | Protocol             |
|-----------|----------------------|
| 0         | OTA                  |
| 4         | crash dump           |
| 5         | CAN bridge           |
| 0xF0–0xFF | reserved (meta)      |
| 0xFF      | capability discovery |

A device-side dispatcher registers the modules it serves; frames for an
unregistered module are silently ignored. A protocol's replies use the **same**
module as its requests (the reply/direction lives in the frame's `flags`, not
the module), so both route to the one registered handler — use `frame.is_reply()`
to distinguish them. In practice a device only *receives* requests (it *sends*
the replies), so its handler normally sees requests only. Application code may
assign any unused module id to its own protocol — nothing is hard-wired to a
specific service.

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

## Capability discovery

A module can be registered with a `ModuleInfo` (name / web app / description) so a
connected peer can ask the device **which** modules it runs — over the reserved
discovery module id `0xFF` — and render or link each one. This powers the browser
**Device Hub** app (`components/dispatcher/web/dispatcher_hub.html`, hosted at
`apps/dispatcher_hub.html`): connect over WebUSB / Web Serial, and it lists the
device's modules as tabs, each linking to that module's own web app.

- `struct ModuleInfo { std::string name, app, description; };`
- `void register_module(uint8_t id, handler_fn handler, ModuleInfo info)` — the
  registration overload that carries metadata (a module with an empty `name` is
  not advertised).
- `void set_device_info(std::string name, std::string firmware = "")` — advertised
  at the head of the reply.
- `std::vector<uint8_t> describe() const` — the serialized capability payload, for
  apps that own their transmit path.
- `void serve_discovery(reply_fn reply)` — opt in to auto-answering the discovery
  query. This is the **only** path by which a Dispatcher sends: it hands the encoded
  reply frame to your transmit callback. The router stays otherwise send-free.

```cpp
dispatcher.set_device_info("espp MCP266 Console", "1.0.0");
dispatcher.register_module(6, mcp_handler,
    {.name = "MCP266", .app = "mcp266_console.html", .description = "Configure & command motors"});
// answer discovery over 0xFF using the app's transport
dispatcher.serve_discovery([&](std::span<const uint8_t> frame) { usb.write_vendor(frame); });
```

The discovery reply payload is a compact binary TLV (all lengths one byte;
strings are `[len][bytes]`): `[version][reserved][device_name][device_fw]
[module_count]` then per module `[id][name][app][description]`. The reserved
discovery module (`0xFF`) never lists itself.

## Host tests

```
c++ -std=c++20 -Werror -I components/dispatcher/include -I components/stream_frame/include \
    components/dispatcher/test/dispatcher_host_test.cpp -o test && ./test
```
