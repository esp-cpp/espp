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
module's handler untouched; the Dispatcher does not interpret them. espp's own
protocols and examples use, for example:

| Module    | Protocol                                       |
|-----------|-------------------------------------------------|
| 0         | OTA                                             |
| 1         | Core-dump example crash trigger (example only)  |
| 2         | BLDC haptics                                    |
| 3         | Telemetry                                       |
| 4         | crash dump                                      |
| 5         | CAN bridge                                      |
| 6         | MCP266                                          |
| 0xF0–0xFE | reserved (meta)                                 |
| 0xFF      | capability discovery                            |

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
- `template <DispatcherModuleConcept Service> void register_module(Service &service)`
  — register a *service* object. The contract is the C++20 concept
  `espp::DispatcherModuleConcept`: `uint8_t module_id() const`, `ModuleInfo
  module_info() const` and `void handle(const stream_frame::Frame&)`, all
  callable on the object (static members work too). `espp::OtaService`,
  `espp::CoreDumpService`, `espp::Telemetry`, `espp::Mcp266Service` satisfy it
  (and `static_assert` so); add `static_assert(espp::DispatcherModuleConcept<MyModule>);`
  to your own module for a precise compile-time check. The id and metadata are
  read from the *object*, so a module whose id is configured per instance
  registers under that id.
- `void feed(std::span<const uint8_t> data)` — parse + route.
- `void dispatch(const stream_frame::Frame&)` — route an already-parsed frame.
- `void reset()` — drop buffered bytes (reconnect / RX overflow).
- `buffered()`, `dropped_bytes()`.

```cpp
espp::Dispatcher dispatcher;
dispatcher.register_module(ota_service);      // espp::OtaService, module 0
dispatcher.register_module(coredump_service); // espp::CoreDumpService, module 4
dispatcher.register_module(0x10, [&](const espp::stream_frame::Frame &f) {
  // your own protocol: f.type, f.is_reply(), f.payload
});
usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { dispatcher.feed(data); });
```

## DispatcherWorker: feeding it from a transport

Transport receive callbacks (the TinyUSB task, a socket reactor, ...) must not
block, while protocol handlers routinely do (an OTA `BEGIN` erases a partition).
`espp::DispatcherWorker` (`dispatcher_worker.hpp`) is a `Dispatcher` plus the
bounded receive queue and `espp::Task` worker that feed it — the plumbing every
espp example used to hand-roll:

```cpp
espp::DispatcherWorker link({.send = [&](std::span<const uint8_t> f) { usb.write_vendor(f); },
                             .on_overflow = [&]() { ota_service.on_rx_overflow(); },
                             .task_config = {.name = "usb_rx", .stack_size_bytes = 8192}});
link.register_module(ota_service);          // handlers run on the worker task
link.register_module(coredump_service);
link.serve_discovery("My Device", version); // 0xFF discovery, replies via `send`
usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { link.push(data); });
```

- `push(bytes)` — queue received bytes (any task; short lock only). Returns
  `false` if they were dropped.
- Overflow (`max_queued_bytes`, default eight max-size frames): everything
  queued is dropped, the parser is reset so it resynchronizes on the next frame
  magic, and `on_overflow` runs on the worker so a protocol can abort an
  in-flight transfer and tell the peer.
- `request_reset()` — discard a half-parsed frame across a transport
  (re)connect; safe from a mount/unmount callback.
- `sender()` — the configured `send`, to hand to services registered on this
  stream; `dispatcher()` — the underlying router (only touch it from a handler
  or before any bytes are pushed).
- One worker per byte stream: vendor + CDC = two workers, each with its own
  `send`, with the services registered on both.

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
