# OTA (Over-the-Air Firmware Update) Component

[![Badge](https://components.espressif.com/components/espp/ota/badge.svg)](https://components.espressif.com/components/espp/ota)

`espp::Ota` is a **transport-agnostic OTA firmware update engine** for ESP-IDF:
an idiomatic espp wrapper around `esp_ota_ops` with a single mutex-serialized
update session, `std::error_code` error reporting (no exceptions), incoming
image introspection, and app-rollback helpers. It performs no I/O of its own —
feed it image bytes from **any** transport (USB vendor / WebUSB stream, HTTP
request body, TCP socket, UART, SD card, ...) and it streams them into the next
OTA app partition.

The component also ships `espp::detail::ota_stream` (in
`include/detail/ota_stream_protocol.hpp`), a **host-testable, ESP-free framed
stream protocol** for carrying OTA over a raw byte stream such as the espp
`usb_device` vendor (WebUSB) interface — with CRC-32-verified frames,
incremental parsing, resynchronization, and a bounded maximum frame size. The
matching browser uploader is
[`web/ota_console.html`](web/ota_console.html), hosted at
[esp-cpp.github.io/espp/apps/ota_console.html](https://esp-cpp.github.io/espp/apps/ota_console.html).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [OTA (Over-the-Air Firmware Update) Component](#ota-over-the-air-firmware-update-component)
  - [Features](#features)
  - [API](#api)
  - [Stream protocol (USB / WebUSB)](#stream-protocol-usb--webusb)
  - [Rollback](#rollback)
  - [Example](#example)
  - [Testing](#testing)

<!-- markdown-toc end -->

## Features

- **Transport-agnostic session**: `begin(image_size)` → `write(chunk)`... →
  `finish()` / `abort()`; `image_size` may be 0 for unknown-length streaming
- **First-chunk validation**: checks the ESP image magic byte (`0xE9`) and
  extracts + logs the incoming `esp_app_desc_t` (project name, version, build
  date), exposed via `incoming_app_description()`; optional
  `reject_same_version` config knob
- **Full validation on finish**: `esp_ota_end()` verifies the complete image
  (including its SHA-256, and signature under secure boot) before
  `esp_ota_set_boot_partition()`; restart is a **separate** `restart()` call so
  the application controls timing
- **Rollback helpers**: `is_pending_verify()`, `mark_app_valid()`,
  `mark_app_invalid_and_rollback()` (require
  `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`)
- **Progress**: optional `progress_callback(written, total)` invoked on every
  write
- **Introspection**: running / boot / update partition labels + sizes, running
  and incoming app descriptions, `session_active()`, `bytes_written()`
- **No exceptions**: every failure is reported via `std::error_code`
- **Host-buildable wire framing**: the USB/WebUSB stream protocol lives in
  `detail/ota_stream_protocol.hpp` and builds with just a C++20 standard
  library

## API

Key class: `espp::Ota` (header-only, `ota.hpp`)
- `begin(size_t image_size, std::error_code&)` — open the next OTA partition
  (`image_size == 0` = unknown / streaming, erases the whole partition)
- `write(std::span<const uint8_t>, std::error_code&)` — stream image bytes; on
  any failure the session is aborted
- `finish(std::error_code&)` — validate the image + set the boot partition
  (does **not** restart)
- `abort(std::error_code&)` — discard the session (idempotent)
- `restart()` — `esp_restart()`, call after a successful `finish()`
- `is_pending_verify()` / `mark_app_valid(ec)` / `mark_app_invalid_and_rollback(ec)`
- `running_partition_label()` / `boot_partition_label()` /
  `update_partition_label()` + `_size()` getters,
  `running_app_description()`, `incoming_app_description()`,
  `session_active()`, `bytes_written()`, `image_size()`

Service class: `espp::OtaService` (`ota_service.hpp`) — the stream protocol
below as a drop-in [dispatcher](../dispatcher) module (module 0 by default), so
an application never implements the OTA state machine itself:

```cpp
auto send = [&](std::span<const uint8_t> frame) { usb.write_vendor(frame); };
espp::Ota ota({.log_level = espp::Logger::Verbosity::INFO});
espp::OtaService ota_service(ota, {.send = send});
espp::DispatcherWorker link({.send = send, .on_overflow = [&] { ota_service.on_rx_overflow(); }});
link.register_module(ota_service); // module 0 + discovery metadata
link.serve_discovery("My Device");
usb.set_vendor_receive_callback([&](std::span<const uint8_t> data) { link.push(data); });
```

- construct with the `Ota` engine and a `send` function; `Config` also has
  `module` (the dispatcher module id the instance answers on and stamps on its
  replies — default `kModule` = 0, which is what the OTA console and the
  `espp_ota` CLI look for; the id is only a routing key, so move it only if
  your host tooling is told the new id), `auto_restart` (default true: reply
  `OK` to `FINISH`, then restart after `restart_delay`, 750 ms) and
  `on_update_finished` (run your own logic / `Ota::restart()` when
  `auto_restart` is off)
- `handle(frame)` — the dispatcher entry point (ignores other modules and
  replies); `feed(bytes)` / `handle_frame(type, payload)` for standalone use;
  `on_rx_overflow()` — abort the transfer and tell the host after the transport
  dropped bytes; `owns_session()`
- **per-transport session ownership**: one `OtaService` per byte stream (they
  may share one `Ota`); each only appends to / finishes / aborts a session *it*
  began, so a `DATA` frame on one transport can never touch a session started
  on another (e.g. the example's HTTP upload)
- rollback stays host-driven (see below): the service answers `GET_STATUS` /
  `MARK_VALID` / `MARK_INVALID` and never marks the running image valid itself
- replies are always sent after the internal lock is released (the same
  contract as `CoreDumpService`)

## Stream protocol (USB / WebUSB)

`detail/ota_stream_protocol.hpp` frames OTA messages over any raw byte stream
(the espp `usb_device` vendor / WebUSB interface in the example). All fields
little-endian:

```
[magic u16 = 0x4F54 "OT"][flags u8][module u8][type u8][len u32][payload...][crc32 u32]
```

This is the shared [`stream_frame`](../stream_frame) v2 codec: `flags` bit0 =
reply (0 = request, 1 = device→host reply) and bits 4-7 = version (1); `module`
is the routing id (OTA is **module 0** by default — every `make_*` builder takes
an optional module argument, and `OtaService::Config::module` moves the
service). OTA layers its message types on it.

- `crc32` is the standard zlib CRC-32 (poly `0xEDB88320` reflected, init/final
  xor `0xFFFFFFFF`) over magic..payload; check value `crc32("123456789") ==
  0xCBF43926`
- payload length is capped at **4096 bytes** — the incremental `StreamParser`
  rejects and resynchronizes past oversized or corrupt frames, so buffering
  stays bounded
- host → device (requests): `0x01 BEGIN(u32 image_size)`, `0x02 DATA(bytes)`,
  `0x03 FINISH`, `0x04 ABORT`, `0x08 GET_STATUS`, `0x09 MARK_VALID`,
  `0x0A MARK_INVALID`; device → host (replies, reply flag set):
  `0x05 OK(u32 bytes_received)`, `0x06 ERROR(u32 code + utf8 message)`,
  `0x07 PROGRESS(u32 written, u32 total)`, `0x0B STATUS(u8 flags + running app
  version + project name)` (flags bit0 = pending-verify, bit1 = rollback-supported;
  each string is u8-length-prefixed)
- transactions are serialized: the host waits for `OK` / `ERROR` (or `STATUS`)
  before the next frame
- **rollback is host-driven** (see below): after an OTA the new image boots
  *pending verify*, and the **host** confirms it with `MARK_VALID` once it has
  checked the device is healthy — the running app must not confirm itself, or a
  broken build could mark itself valid before failing. `MARK_INVALID` rolls back
  to the previous image and reboots; `GET_STATUS` reports whether the running
  image is still pending verify.

The [espp OTA Console](https://esp-cpp.github.io/espp/apps/ota_console.html)
(`web/ota_console.html`) implements this protocol over WebUSB in the browser.

### Command line: build → OTA

The [`python/espp_ota`](python/) tool speaks the same protocol from a terminal.
Because this component ships an `idf_ext.py`, any project using it gets an
`idf.py ota-usb` action — the OTA counterpart to `idf.py flash`, with options
(`--no-verify`, `--binary`, `--status` / `--mark-valid` / `--rollback`,
`--vid`/`--pid`/`--serial`, ...); the same action is also available through an
`idf_extension` entry point of the espp wheel, and `project_include.cmake`
keeps a plain `ota-usb` CMake target as a fallback:

```sh
pip install pyusb          # once (needs a libusb backend)
idf.py ota-usb            # builds the app, then OTAs it over USB (+ marks it valid)
idf.py ota-usb --status   # query the rollback state instead of flashing
```

Or drive it directly: `python -m espp_ota flash build/<app>.bin` (see
[`python/README.md`](python/README.md)).

## Rollback

With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, a freshly-installed app boots in
the `ESP_OTA_IMG_PENDING_VERIFY` state and the bootloader rolls it back to the
previous image on the next reset **unless it is confirmed**
(`mark_app_valid()`). `mark_app_invalid_and_rollback()` actively rejects the new
image and reboots into the previous one. The engine only provides these
primitives; **who** calls them, and **when**, is a policy the application picks:

- **Device self-validation**: the app runs its own health checks at boot and
  calls `mark_app_valid()` itself. Simple, but a broken build can validate
  itself right before failing (the check may not catch what breaks it).
- **Host-driven confirmation** (what this example + tooling do): the app does
  **not** confirm itself — it stays pending and exposes `GET_STATUS` /
  `MARK_VALID` / `MARK_INVALID` over the stream protocol, and the **host**
  confirms the image once it has verified the device is healthy (the
  [OTA Console](https://esp-cpp.github.io/espp/apps/ota_console.html) and the
  `espp-ota` CLI do this on reconnect). More robust: an image that cannot boot
  and answer the host is never confirmed, so it rolls back.

See the [example](./example) for the host-driven wiring.

## Example

The [example](./example) wires **three transports** to the same `espp::Ota`
engine on an ESP32-S3:

- **USB vendor / WebUSB** using `espp::UsbDevice` + the stream protocol above
  (update from the browser via `web/ota_console.html`)
- **WiFi HTTP push**: `POST /ota` streams the request body into the engine
  (`curl --data-binary @firmware.bin http://<ip>/ota`), and `GET /ota` serves a
  tiny browser upload page
- the **same HTTP server works unchanged over Ethernet** (the espp `ethernet`
  component / any `esp_netif`)

## Testing

The wire framing is host-tested (no ESP-IDF needed):

```bash
c++ -std=c++20 -Werror -I components/ota/include -I components/stream_frame/include \
    components/ota/test/ota_protocol_host_test.cpp -o ota_test && ./ota_test
```

It covers every request builder + reply parser, including the status / rollback
messages (`GET_STATUS` / `MARK_VALID` / `MARK_INVALID`, `make_status` /
`parse_status`) and malformed / truncated `STATUS` payloads.
