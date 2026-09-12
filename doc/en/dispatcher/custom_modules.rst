Building Custom Modules & Protocols
************************************

This page is an in-depth, worked tutorial for adding **your own** protocol to
an espp device: a device-side *module* that speaks a message set you define,
multiplexed onto a shared USB link alongside espp's own protocols (OTA, crash
dump, ...), plus a browser web app that talks to it. It assumes you have read
:doc:`../stream_frame/index` and :doc:`dispatcher` (the API reference for
`espp::Dispatcher`) already; this page is the "how do I use these to ship a
feature" companion to those API docs.

Concepts: the layers
=====================

A custom protocol sits on three layers, each with a single, narrow job:

.. mermaid::

   flowchart TD
     subgraph browser["Browser web app"]
       UI["your module's console (module_console.html)"]
     end
     subgraph device["ESP32 device"]
       T["espp::UsbDevice\n(vendor bulk IN/OUT, or CDC)"]
       D["espp::Dispatcher\n(routes by module id, serves discovery on 0xFF)"]
       M["your module\n(Config{send_fn} + build() + feed()/handle())"]
       OTA["espp::CoreDumpService / OTA / ... (other modules)"]
       T -- "write_vendor() / write_cdc()" --> UI
       UI -- "WebUSB transferOut / Web Serial write" --> T
       T -- "on_receive callback" --> D
       D -- "module id N" --> M
       D -- "module id 4, 0, ..." --> OTA
     end

- **stream_frame** (:doc:`../stream_frame/index`) is the wire *framing*: it
  turns a payload into a CRC-32-verified, length-delimited frame carrying a
  routing ``module`` byte, a ``type`` byte, and a reply/request ``flags`` bit
  — and turns a raw byte stream back into frames. It has no idea what any
  module means; it only builds and parses frames.
- **Dispatcher** (this component) parses that stream **once** and routes each
  frame to the handler registered for its ``module`` id, so several protocols
  (yours and espp's) can share one stream without each running its own parser.
  It also answers a reserved *discovery* query (module ``0xFF``) so a peer can
  ask which modules a device runs — see `Discovery + the webapp side`_ below.
- **The transport** moves the encoded bytes between device and host: a USB
  **vendor** (WebUSB) interface, a USB **CDC** (Web Serial) interface, a
  socket, or a UART. `espp::Dispatcher` and `stream_frame` don't care which —
  they only need a `send` function and a stream of received bytes. This guide
  focuses on `espp::UsbDevice`'s vendor interface, the transport used by every
  shipped espp module console.

Module ids already in use
--------------------------

The ``module`` byte is a full byte (0..255), so up to 256 protocols can
coexist on one stream. espp's own protocols and examples currently claim:

=========  ========================================================
Module id  Protocol
=========  ========================================================
0          OTA (firmware update)
1          Core-dump example crash trigger (example only)
2          BLDC haptics (``components/bldc_haptics``)
3          Telemetry (``espp::Telemetry``)
4          Crash dump (``espp::CoreDumpService``)
5          CAN bridge (``components/canopen``)
6          MCP266 motor-controller console
0xF0-0xFE  reserved for dispatcher / meta use
0xFF       capability discovery
=========  ========================================================

Pick any id **not** in this table (or not already used by other modules in
your own application) for your protocol. Nothing in `espp::Dispatcher` is
hard-wired to a specific id — a device only ever registers the modules it
actually serves, and frames for an unregistered id are silently ignored.

Several modules can, and routinely do, share **one** `Dispatcher` over **one**
USB link. The `bldc_haptics` example registers OTA, the crash-dump service,
*and* its own haptics protocol on a single dispatcher instance
(``components/bldc_haptics/example/main/bldc_haptics_example.cpp``):

.. code-block:: cpp

   dispatcher.register_module(
       otap::kModule,
       [&](const espp::stream_frame::Frame &frame) {
         if (!frame.is_reply())
           handle_ota_frame(frame);
       },
       {.name = "OTA", .app = "ota_console.html", .description = "Firmware update over USB"});
   dispatcher.register_module(espp::CoreDumpService::kModule,
                              [&](const espp::stream_frame::Frame &frame) {
                                if (!frame.is_reply())
                                  coredump_service.handle_frame(frame.type, frame.payload);
                              },
                              {.name = "Core Dump",
                               .app = "coredump_console.html",
                               .description = "Inspect the last crash core dump"});
   dispatcher.register_module(proto::kModule,
                              [&](const proto::stream::Frame &frame) {
                                if (!frame.is_reply())
                                  handle_frame(frame);
                              },
                              {.name = "BLDC Haptics",
                               .app = "haptics_console.html",
                               .description = "Haptic detent / feedback modes"});

Your application module registers alongside these the same way — pick an
unused id, register a handler, and (optionally) attach `ModuleInfo` so it is
discoverable (see `Discovery + the webapp side`_).

Anatomy of a module: the ``CoreDumpService`` pattern
=====================================================

`espp::CoreDumpService` (``components/coredump/include/coredump_service.hpp``)
is the best reference for the *shape* a device-side module takes, because it
is a complete, production module: it owns a protocol (module id, message
types), is transport-agnostic, and is registered on a `Dispatcher` exactly
like your own module will be. The pattern has five parts:

1. **A module id.**

   .. code-block:: cpp

      /// Dispatcher module id owned by the core-dump protocol (the frame `module`
      /// byte). Reply Msg values keep the high bit set, which build() maps to the
      /// frame reply flag.
      static constexpr uint8_t kModule = 4;

2. **Message types**, as a scoped enum of `uint8_t` values. `CoreDumpService`
   uses one *convention* — request types in ``0x40``-``0x43``, and their reply
   counterparts at ``0xC0``-``0xC4`` (the high bit set) — purely so `build()`
   can derive the frame's `reply` bit from the type value instead of taking it
   as a separate parameter:

   .. code-block:: cpp

      enum class Msg : uint8_t {
        // host -> device
        GetSummary = 0x40, ///< request the crash report text
        GetSize = 0x41,    ///< request the core-dump image size
        Read = 0x42,       ///< read image bytes (u32 offset + u16 length)
        Erase = 0x43,      ///< erase the stored core dump
        // device -> host
        Summary = 0xC0, ///< UTF-8 crash report (empty = clean boot history)
        Size = 0xC1,    ///< u32 image size (0 = no core dump)
        Data = 0xC2,    ///< u32 offset + image bytes
        Ok = 0xC3,      ///< u32 context-dependent success value
        Error = 0xC4,   ///< u32 informational code + authoritative UTF-8 message
      };

   This is a convention, **not** something `stream_frame` or `Dispatcher`
   requires — the authoritative reply/request signal is always the frame's
   `flags` bit0 (see `Frame::is_reply()` and `build_frame(bool reply, ...)`),
   which every module must set correctly regardless of how it lays out its
   `type` values. Other espp modules pick different layouts for the same
   underlying rule: `espp::Telemetry` (module 3) uses request types ``0x0X``
   and reply types ``0x8X``; the MCP266 console protocol
   (``components/mcp266/webapp_example/main/mcp266_protocol.hpp``, module 6)
   uses high-nibble ``0x6_`` for requests and ``0xE_`` for replies. Any layout
   works as long as your `build()` (or equivalent) passes the right `reply`
   bool to `build_frame()`.

3. **A `Config` carrying a `send_fn`.** The module never touches a transport
   directly — it is handed a callback that transmits one already-encoded
   frame, so the same module class works unmodified over USB vendor, USB CDC,
   a socket, or a UART:

   .. code-block:: cpp

      using send_fn = std::function<void(std::span<const uint8_t> frame)>;

      struct Config {
        send_fn send{nullptr}; ///< Transmits an encoded reply frame (required).
        espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
      };

   Mounting it on a transport is then a one-liner at construction time (from
   ``components/coredump/example/main/coredump_example.cpp``):

   .. code-block:: cpp

      espp::CoreDumpService vendor_service(
          core_dump, {.send = [&](std::span<const uint8_t> frame) { usb.write_vendor(frame); },
                      .log_level = espp::Logger::Verbosity::INFO});

4. **A `build()` helper** that wraps `stream_frame::build_frame()` so every
   reply in the module goes through one place:

   .. code-block:: cpp

      static std::vector<uint8_t> build(Msg type, std::span<const uint8_t> payload = {}) {
        namespace stream = espp::stream_frame;
        const bool reply = (static_cast<uint8_t>(type) & 0x80) != 0;
        return stream::build_frame(reply, kModule, static_cast<uint8_t>(type), payload);
      }

5. **`feed()` / `handle_frame()` entry points** that parse (or accept an
   already-parsed) frame, build a reply payload, and only *then* invoke
   `send`. `CoreDumpService::handle_frame_locked()` runs the request under an
   internal mutex, but note carefully how the reply is delivered:

   .. code-block:: cpp

      bool handle_frame(uint8_t type, std::span<const uint8_t> payload) {
        std::vector<uint8_t> reply;
        bool handled;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          handled = handle_frame_locked(type, payload, reply);
        }
        // send outside the lock so a re-entrant transport cannot deadlock
        if (!reply.empty())
          send(reply);
        return handled;
      }

   The reply is always sent **after** releasing the module's internal lock.
   This matters the moment your `send` callback might call back into the
   module (a loopback transport in a host test, or an error path that resets
   the parser) — sending under the lock would deadlock. Copy this shape even
   for a module simple enough that it seems unnecessary; it costs nothing and
   avoids a subtle bug later.

   `handle_frame_locked()` itself is a plain switch over your `Msg` enum,
   building a reply payload with the `stream_frame::put_*` helpers and
   returning `false` for any `type` outside the module — which is what lets
   several protocols share a stream (`feed()`/`Dispatcher::feed()` simply
   moves on for module ids or types the handler doesn't own):

   .. code-block:: cpp

      case Msg::GetSize: {
        const size_t raw_size = core_dump_.image_size();
        const auto size =
            static_cast<uint32_t>(std::min<size_t>(raw_size, std::numeric_limits<uint32_t>::max()));
        std::vector<uint8_t> reply_payload;
        stream::put_u32(reply_payload, size);
        reply = build(Msg::Size, reply_payload);
        return true;
      }

   For errors, `CoreDumpService::build_error()` packs a `u32` code plus a
   UTF-8 message and is a good template for your own error replies: treat the
   message as authoritative and the code as best-effort/informational (see
   the header comment in ``coredump_service.hpp`` for why — `std::errc`
   numbering is not portable across C++ standard libraries).

A minimal "hello" module, from scratch
=======================================

`CoreDumpService` is a *complete* module (flash access, a mutex, error
mapping); the pattern above is more machinery than a small application
protocol needs. Here is the same five-part shape reduced to the minimum: a
module that answers a ``PING`` request (an arbitrary string payload) with a
``PONG`` reply (the string, upper-cased), built directly from
`espp::stream_frame` and `espp::Dispatcher` — no protocol header, no internal
parser, no mutex, because a handler this small can run straight out of the
`Dispatcher`'s own frame:

.. code-block:: cpp

   #include <algorithm>
   #include <cctype>
   #include <cstdint>
   #include <functional>
   #include <span>
   #include <string>
   #include <vector>

   #include "dispatcher.hpp"
   #include "stream_frame.hpp"

   namespace hello_module {
   // 1. Module id: pick one not already in use (see the table above).
   static constexpr uint8_t kModule = 0x10;
   // 2. Message types: PING (host->device) and PONG (device->host, high bit
   //    set) — purely a naming convention, same as CoreDumpService's Msg.
   enum class Msg : uint8_t { Ping = 0x00, Pong = 0x80 };
   } // namespace hello_module

   class HelloModule {
   public:
     // 3. Config carrying a send_fn: the module never touches USB/UART/socket
     //    APIs directly.
     using send_fn = std::function<void(std::span<const uint8_t> frame)>;
     struct Config {
       send_fn send{nullptr};
     };
     explicit HelloModule(const Config &config) : send_(config.send) {}

     // 5. Entry point: register this directly as the Dispatcher handler for
     //    hello_module::kModule (skips the feed()/handle_frame() split that
     //    CoreDumpService needs so it can also run standalone off a raw byte
     //    stream — feed() owns an internal parser for that case — and
     //    serialize flash access under its own mutex; a handler this small
     //    has neither concern).
     void handle(const espp::stream_frame::Frame &frame) {
       if (frame.is_reply() || frame.type != static_cast<uint8_t>(hello_module::Msg::Ping))
         return; // not a request we answer (ignore replies / other types)
       std::string text(frame.payload.begin(), frame.payload.end());
       // Payload bytes are arbitrary, not guaranteed ASCII text, so cast to
       // unsigned char before calling ::toupper (it's UB on a negative
       // signed-char value); non-ASCII bytes simply pass through unchanged.
       std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
         return static_cast<char>(::toupper(c));
       });
       const auto reply_frame = build(hello_module::Msg::Pong, std::span<const uint8_t>(
                                                                    reinterpret_cast<const uint8_t *>(
                                                                        text.data()),
                                                                    text.size()));
       send_(reply_frame);
     }

   private:
     // 4. build(): every reply goes through stream_frame::build_frame(); the
     //    Msg high bit (see the enum above) selects the reply flag, exactly
     //    like CoreDumpService::build().
     static std::vector<uint8_t> build(hello_module::Msg type, std::span<const uint8_t> payload) {
       const bool reply = (static_cast<uint8_t>(type) & 0x80) != 0;
       return espp::stream_frame::build_frame(reply, hello_module::kModule,
                                              static_cast<uint8_t>(type), payload);
     }

     send_fn send_;
   };

Note the reply is bound to a local `reply_frame` before it is handed to `send_`:
`send_fn`'s `std::span<const uint8_t>` parameter is only valid for the
duration of that one call, so a `send` implementation that needs to keep the
bytes past the call (queueing it for a later retry, for example) must copy
them rather than retain the span.

Wiring it up looks exactly like any other module — construct it with a `send`
that writes to your transport, then register it (with `ModuleInfo` so it is
discoverable — see below):

.. code-block:: cpp

   HelloModule hello({.send = [&](std::span<const uint8_t> frame) { usb.write_vendor(frame); }});
   dispatcher.register_module(
       hello_module::kModule, [&](const espp::stream_frame::Frame &f) { hello.handle(f); },
       {.name = "Hello", .app = "hello_console.html", .description = "PING/PONG demo module"});

That's a complete, working custom protocol: a module id, two message types, a
`send_fn`-based `Config`, a `build()` helper, and a handler registered on a
`Dispatcher`. Everything in the rest of this page — richer payloads,
transports, and discovery — builds on exactly this shape.

Defining a protocol / payload
==============================

`stream_frame` fixes the *frame* layout (magic, flags, module, type, len,
payload, CRC — see :doc:`../stream_frame/index`); the *payload* layout is
entirely up to your protocol. A few conventions used by every espp module
keep payloads simple to parse on both the device (C++) and the browser
(JavaScript `DataView`):

- **Little-endian, fixed-width fields.** The frame header itself is
  little-endian (see the wire-format comment at the top of
  ``stream_frame.hpp``), and every espp module payload follows the same rule
  for multi-byte fields.
- **The `put_u16` / `put_u32` / `get_u16` / `get_u32` helpers** append or read
  a little-endian integer:

  .. code-block:: cpp

     inline void put_u16(std::vector<uint8_t> &out, uint16_t value);
     inline void put_u32(std::vector<uint8_t> &out, uint32_t value);
     inline uint16_t get_u16(std::span<const uint8_t> bytes);
     inline uint32_t get_u32(std::span<const uint8_t> bytes);

  Strings are commonly length-prefixed with a single byte, as
  `Dispatcher::describe()` does for its discovery payload (see below) —
  ``[len u8][bytes]``, truncated at 255 bytes, so a reader never needs to
  scan for a terminator.
- **The `reply` bit is a `build_frame()` parameter, not a payload field.**
  Whether a frame is a request or a response/event lives in the frame
  `flags` (bit0), set via `build_frame(bool reply, uint8_t module, uint8_t
  type, payload, correlation)`. Folding the direction into your `type` enum
  (as `Msg::Summary = 0xC0` does) is a convenience for `build()`, not a wire
  requirement — `Dispatcher` and `stream_frame` only ever look at the `flags`
  bit.
- **An optional `correlation` id** (`std::optional<uint16_t>`, the last
  `build_frame()` parameter) lets a protocol match a reply to the request
  that triggered it when more than one request may be outstanding at once.
  Single-request-in-flight protocols (most espp modules, including
  `CoreDumpService`) don't need it and simply omit the argument.
- **`stream_frame::kMaxPayloadSize` (4096 bytes)** bounds one frame's
  payload; `build_frame()` returns an empty vector if you exceed it (check
  for that, as `Dispatcher::serve_discovery()` does, rather than sending
  nothing silently).
- **Add typed helpers when you need them.** `stream_frame` ships only
  integer helpers; a protocol that carries floats defines its own, the same
  way `espp::Telemetry` (module 3, ``components/telemetry/include/telemetry.hpp``)
  does for its `SAMPLE` frames:

  .. code-block:: cpp

     /// Append a little-endian IEEE-754 float32 to a byte buffer.
     static void put_f32(std::vector<uint8_t> &out, float value) {
       uint32_t bits;
       std::memcpy(&bits, &value, sizeof(bits));
       espp::stream_frame::put_u32(out, bits);
     }

  used exactly like the built-in helpers when building a payload:

  .. code-block:: cpp

     std::vector<uint8_t> p;
     espp::stream_frame::put_u32(p, timestamp_us);
     for (float v : values)
       put_f32(p, v);
     frame = build(Type::Sample, p);

  On the browser side the matching read is one `DataView.getFloat32(offset,
  true)` call (the `true` selects little-endian) — no bit-reinterpretation
  needed there, since JavaScript's `DataView` already has a native float
  accessor.

The USB vendor transport
==========================

Every shipped espp module console reaches the device over
`espp::UsbDevice`'s **vendor** function (a `bInterfaceClass 0xFF` interface
with one bulk IN + one bulk OUT endpoint) advertising **WebUSB**, so a
Chromium-based browser can open it with no driver install. The same protocol
usually also rides the **CDC** function (Web Serial) so it can share a stream
with human-readable console text.

Enabling the vendor interface (device side)
--------------------------------------------

.. code-block:: cpp

   espp::UsbDevice::Config usb_cfg;
   usb_cfg.pid = 0x0000; // replace with your project's allocated PID (e.g. from
                         // pid.codes) so a webapp's WebUSB filter can find your
                         // device specifically — this placeholder is not a real
                         // assignment
   usb_cfg.manufacturer = "espp";
   usb_cfg.product = "My espp Device";

   espp::UsbDevice::VendorFunction vendor;
   vendor.webusb = true; // advertise BOS / WebUSB / MS OS 2.0 descriptors
   vendor.landing_page_url = "esp-cpp.github.io/espp/apps/hello_console.html";
   usb_cfg.vendor = vendor;

   espp::UsbDevice usb(usb_cfg);
   std::error_code ec;
   if (!usb.initialize(ec)) { /* handle ec */ }

`VendorFunction` (``components/usb_device/include/usb_device.hpp``) also
exposes `on_receive` (a callback invoked with received bytes — the same thing
`set_vendor_receive_callback()` sets after construction), `rx_chunk_size`, and
the raw WebUSB control-request fields (`url_scheme`, `webusb_vendor_code`,
`ms_os_vendor_code`) if you need to customize them. Building the vendor class
into the firmware requires enabling it in `esp_tinyusb`'s Kconfig
(``sdkconfig.defaults``)::

   CONFIG_TINYUSB_VENDOR_COUNT=1   # compiles in the TinyUSB vendor class driver

Sending and receiving
-----------------------

- `usb.write_vendor(data, ec)` (or the exception-free overload
  `usb.write_vendor(data)`) queues bytes on the vendor bulk IN endpoint — this
  is what your module's `send_fn` calls. Writes are **all-or-nothing**: a
  frame that fits the TX FIFO is either queued whole or not sent at all (see
  the doc comment on `write_vendor()` for the exact backpressure contract),
  so a reader on the other end never sees a truncated frame.
- `usb.set_vendor_receive_callback(cb)` (or `VendorFunction::on_receive` at
  construction) delivers received bytes from the TinyUSB device task. Feed
  them straight into your `Dispatcher`:

  .. code-block:: cpp

     usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) {
       dispatcher.feed(data);
     });

  Receive callbacks run in the TinyUSB task and must stay short and
  non-blocking. If a module's handler can block for more than a
  few milliseconds (flash access, a CANopen SDO round-trip, ...), queue the
  bytes and call `dispatcher.feed()` from a worker task instead — see the
  RX-queueing pattern in
  ``components/coredump/example/main/coredump_example.cpp`` (a bounded
  `std::deque` drained by an `espp::Task`, with `dispatcher.reset()` called
  when a chunk had to be dropped so a frame straddling the gap
  resynchronizes at once).
- One `Dispatcher` per byte stream. If you enable both vendor and CDC, use
  **two** `Dispatcher` instances (one per transport) so a frame split across
  reads on one never gets stitched onto bytes from the other — every
  multi-transport espp example (`coredump_example.cpp`, `can_bridge_example.cpp`,
  the MCP266 webapp example) follows this rule.

The CDC (Web Serial) function carries the exact same framed protocol using
`usb.write_cdc()` / `usb.set_cdc_receive_callback()` in place of the vendor
calls above. Because `stream_frame`'s parser resynchronizes on its magic
bytes, plain console text and framed messages can share one CDC stream
without interfering — the core-dump web console relies on exactly this to
double as both a serial monitor and a structured console.

The browser side (WebUSB)
---------------------------

A web app claims the vendor interface, then exchanges raw byte arrays with
it. The Device Hub app (``components/dispatcher/web/dispatcher_hub.html``)
implements this in a small, dependency-free `UsbTransport` class that every
module console can reuse verbatim:

.. code-block:: javascript

   class UsbTransport {
     async open(anyDevice) {
       const options = anyDevice ? { filters: [] } : { filters: [{ vendorId: DEFAULT_VID }] };
       this.device = await navigator.usb.requestDevice(options);
       await this.device.open();
       if (this.device.configuration === null) await this.device.selectConfiguration(1);
       // ... find the interface whose interfaceClass === 0xFF (VENDOR_CLASS)
       //     with a bulk IN + bulk OUT endpoint pair, then claimInterface() it.
     }
     async send(bytes) {
       let sent = 0;
       while (sent < bytes.length) {
         const out = await this.device.transferOut(this.epOut, bytes.subarray(sent));
         if (out.status !== "ok") throw new Error("OUT transfer status: " + out.status);
         const n = out.bytesWritten || 0;
         if (n === 0) throw new Error("OUT transfer made no progress");
         sent += n;
       }
     }
     async readLoop(onData) {
       while (this.reading) {
         const result = await this.device.transferIn(this.epIn, MAX_FRAME);
         if (result.data && result.data.byteLength)
           onData(new Uint8Array(result.data.buffer, result.data.byteOffset, result.data.byteLength));
       }
     }
   }

Every shipped module console (``coredump_console.html``, ``mcp266_console.html``,
``can_bridge_console.html``, ``ds402_panel.html``, the BLDC haptics webapp,
``telemetry.html``) carries its own copy of the same three building blocks —
a `crc32()` matching `stream_frame::crc32()` (golden value
``crc32("123456789") === 0xCBF43926``), a `buildFrame(module, type, payload)`
encoder matching `stream_frame::build_frame()`, and a parser matching
`stream_frame::StreamParser`'s framing/CRC/resync logic — but they are
**wire-compatible variants, not byte-for-byte copies**, so check before you
paste one in wholesale:

- ``dispatcher_hub.html``, ``mcp266_console.html``, and ``ds402_panel.html``
  correctly skip the optional correlation-id bytes when sizing a frame, but
  their parsed frame objects (``{module, type, reply, payload}``) do not
  surface the id — copying one of these verbatim silently drops the
  correlation id this guide recommends for request/reply matching.
- ``coredump_console.html``, ``can_bridge_console.html``, and the BLDC
  haptics console *do* surface it (``{..., correlation, ...}``, ``null``
  when the flag is clear).
- ``telemetry.html`` uses differently-named helpers (`sfCrc32`, `sfBuild`,
  `SfParser` instead of `crc32`, `buildFrame`, `StreamParser`) and, like the
  hub, does not surface the correlation id.
- ``coredump_console.html``'s parser additionally returns every skipped
  (non-frame) byte as a ``text`` field alongside ``frames``, since it shares
  one CDC stream with a plain-text serial console — the other consoles don't
  need or have this.

If you're using correlation ids, copy the parser from ``coredump_console.html``
or ``can_bridge_console.html``; otherwise ``dispatcher_hub.html`` is the
simplest starting point. Whichever you copy, it's deliberately
dependency-free (no build step, no CDN) so it drops straight into a
single-file app, and keeping the codec logic byte-identical to the C++ side
is what makes the CRC and framing agree.

Discovery + the webapp side
==============================

A device doesn't have to hard-code which webapp goes with which module: it
can **advertise** its modules, and a generic hub app can discover and link
them.

Advertising a module
----------------------

Pass a third argument to `register_module()`:

.. code-block:: cpp

   struct ModuleInfo {
     std::string name;        ///< Human-readable module name, e.g. "MCP266 Console".
     std::string app;         ///< Hosted web-app filename, e.g. "mcp266_console.html" (optional).
     std::string description; ///< One-line description (optional).
   };

   void register_module(uint8_t module_id, handler_fn handler, ModuleInfo info);

A module with an empty `name` is not advertised (the plain two-argument
`register_module()` overload defaults to this — useful for a module you want
routed but not shown in a hub UI). `app` must be the exact filename of your
module's hosted web app (see `Hosting your webapp`_ below); the hub treats it
as a same-directory relative link.

Answering discovery
----------------------

Two calls, made once per `Dispatcher` instance, are all a device needs to
answer capability queries:

.. code-block:: cpp

   dispatcher.set_device_info(usb_cfg.product /* , firmware version */);
   dispatcher.serve_discovery([&](std::span<const uint8_t> frame) { usb.write_vendor(frame); });

`serve_discovery()` registers a handler on the reserved discovery module id
`Dispatcher::kDiscoveryModule` (`0xFF`) that answers a
`Dispatcher::Discovery::ListModules` request with `describe()` — a compact
binary payload: `[version u8][reserved u8][device_name str][device_fw str]
[module_count u8]` then, per advertised module, `[id u8][name str][app
str][desc str]` (each `str` is `[len u8][bytes]`). This is the **only** path
by which a `Dispatcher` ever transmits on its own — it stays otherwise a pure
router, and `serve_discovery()` is opt-in.

The Device Hub
-----------------

``components/dispatcher/web/dispatcher_hub.html`` (hosted as
``apps/dispatcher_hub.html``) connects over WebUSB or Web Serial, sends a
`ListModules` request on module `0xFF`, decodes the reply, and renders one tab
per module with a link to its `app`. It validates `app` before ever using it
as a link target — only a bare filename ending in ``.html`` (no scheme, no
slash, no ``..``) is accepted, so a compromised or malicious device cannot
make the hub open a `javascript:`/`data:`/external URL:

.. code-block:: javascript

   function safeAppName(app) {
     return typeof app === "string" && /^[A-Za-z0-9._-]+\.html$/.test(app) && !app.includes("..");
   }

Your module's own console additionally re-runs the same discovery query
itself on connect (every shipped console does) so it can confirm its module
is actually present on the device it just connected to, independent of
whether the user arrived via the hub or opened the console directly.

Hosting your webapp
-----------------------

Any single self-contained HTML file placed at
``components/<your_component>/web/<name>.html`` is hosted automatically by
the docs build alongside every other espp app (see :doc:`../web_apps`) — no
registry entry to add. The apps landing page lists it using its `<title>` and
`<meta name="description">` tags. Keep it dependency-free (no CDN resources,
inline CSS/JS) so it also works fully offline from a `file://` URL and under
GitHub Pages' strict hosting. Make sure the filename you host it under is the
exact string you pass as `ModuleInfo::app`.

Checklist: shipping a new module + webapp
============================================

#. **Pick a module id** not already listed in the `Module ids already in use`_
   table (and not used elsewhere in your own application).
#. **Define your message `type`s** as a small enum; decide (and document) how
   request vs. reply is signalled in your `type` values, remembering the
   *authoritative* signal is always the frame's `flags` reply bit passed to
   `build_frame()`.
#. **Implement the module**: a `Config` carrying a `send_fn`, a `build()`
   helper wrapping `stream_frame::build_frame()`, and either a direct
   `Dispatcher` handler (the "hello" pattern) or your own `feed()` /
   `handle_frame()` split if the module needs its own parser/locking
   (the `CoreDumpService` pattern) — always send replies **after** releasing
   any internal lock.
#. **Register it** on your `Dispatcher` with `ModuleInfo{name, app,
   description}` so it's discoverable; if you support both vendor and CDC,
   register it (with the same `ModuleInfo`) on **both** dispatchers.
#. **Call `set_device_info()` and `serve_discovery()`** once per `Dispatcher`
   / transport pair so a hub (or your own console) can find the device and
   its modules.
#. **Enable the transport**: `CONFIG_TINYUSB_VENDOR_COUNT=1` (and/or the CDC
   equivalent) in ``sdkconfig.defaults``, a `VendorFunction` (and/or
   `CdcFunction`) in your `UsbDevice::Config`, and a receive callback that
   feeds your `Dispatcher`.
#. **Write the webapp**: copy the `crc32()` / `buildFrame()` / `StreamParser`
   JS block from `dispatcher_hub.html` (or any module console), add a small
   UI that builds/decodes your payloads, and host it at
   ``components/<your_component>/web/<name>.html`` — the exact string you put
   in `ModuleInfo::app`.
#. **Test on a host** before touching hardware: `Dispatcher` and
   `stream_frame` are header-only and dependency-free, so a module built on
   them can be exercised in a plain host-side C++ test — see
   ``components/dispatcher/test/dispatcher_host_test.cpp`` and
   ``components/stream_frame/test/stream_frame_host_test.cpp`` for the
   pattern (build and run with a plain ``c++ -std=c++20``, no ESP-IDF
   toolchain needed).

.. seealso::

   - :doc:`../stream_frame/index` — the frame codec this page builds on.
   - :doc:`dispatcher` — the `Dispatcher` API reference.
   - :doc:`../coredump/coredump` — the `CoreDumpService` worked example used
     throughout this page.
   - :doc:`../telemetry/telemetry` — a module streaming typed float samples
     (the `put_f32` example above).
   - :doc:`../buses/canopen` — a bridge module (module 5) plus an
     in-browser CANopen/DS402 client.
   - :doc:`../buses/usb_cdc` — the `espp::UsbDevice` component reference
     (vendor / CDC / HID / X-Input functions, endpoint budgeting).
   - :doc:`../web_apps` — how a component's ``web/`` directory becomes a
     hosted app, and the full list of module consoles shipped today.
