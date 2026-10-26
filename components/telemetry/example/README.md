# Telemetry — USB example (→ Serial Plotter web app)

Streams synthetic float channels from an ESP32-S3 to the browser **Serial
Plotter** web app over USB, using `espp::Telemetry` (a binary telemetry emitter
carried on the `stream_frame` framing, dispatcher module 3 by default).

The hosted app — <https://esp-cpp.github.io/espp/apps/telemetry.html> —
connects on the **vendor (WebUSB)** interface, reads the channel **schema**, and
plots the live **sample** stream. It is the binary, higher-rate,
device-timestamped counterpart to the app's text/CSV Web Serial transport.

## What it does

- Declares four channels — `sine`, `cosine`, `noise`, `ramp` — as the schema.
- A producer task emits one sample (a `float` per channel) every ~10 ms (100 Hz),
  timestamped with the device clock.
- Exposes the stream over the USB **vendor (WebUSB)** interface; a `Dispatcher`
  routes the emitter's module (`Telemetry::Config::module`, 3 by default; only a
  routing key, the hosted Serial Plotter finds it through discovery by its
  protocol id) to it and serves capability discovery so the browser **Device
  Hub** lists this device and links to `telemetry.html`.
- The web app can pause/resume the stream and request a rate (`SET_STREAM`), and
  requests the schema on connect (`GET_SCHEMA`).

`espp::Telemetry` itself is transport-agnostic (the `stream_frame` framing works
over CDC / UART / a socket too); this example streams over WebUSB because that is
what the web app's binary path consumes.

Swap the synthetic generator for your real signals: build a `std::array<float, N>`
in channel order and call `telemetry.emit(...)`.

## Build & run

```sh
idf.py -p /dev/ttyACM0 flash          # target esp32s3 (set in sdkconfig.defaults)
idf.py -p /dev/ttyUSB0 monitor        # console is on UART0 (USB-UART adapter)
```

Then open the Serial Plotter web app, click **Connect (USB)**, and pick the
"espp Serial Plotter" device. The system console/logs go to **UART0**: on the
S3 / P4, USB-Serial-JTAG shares the native USB port's PHY with USB-OTG, which
TinyUSB takes over.

## Standard USB services

Like every espp USB example, this one serves the standard service set on its
framed USB link(s) next to its own protocol, so the hosted consoles and the
[Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html) (which
finds each service through discovery, by protocol id) work against it:

| Service | Module (default) | Protocol id | Console |
|---|---|---|---|
| `espp::SystemService` -- device info, reboot, reboot into the bootloader | 7 | `espp.system` | [system console](https://esp-cpp.github.io/espp/apps/system_console.html) |
| `espp::MonitorService` -- heap regions + task table, on request or streamed | 8 | `espp.monitor` | system console |
| `espp::OtaService` -- firmware update (host-driven rollback confirmation) | 0 | `espp.ota` | [OTA console](https://esp-cpp.github.io/espp/apps/ota_console.html) |
| `espp::CoreDumpService` -- last-crash report, core dump download / erase | 4 | `espp.coredump` | [coredump console](https://esp-cpp.github.io/espp/apps/coredump_console.html) |

`partitions.csv` therefore carries the OTA layout (`otadata`, `ota_0`, `ota_1`)
plus a `coredump` partition, and `sdkconfig.defaults` enables core dumps to
flash, OTA rollback and the FreeRTOS run-time statistics the task monitor
reads. Every device->host write on a transport goes through one mutex, so the
services (and any streaming) never interleave frames.

## Notes

- Native USB (vendor / WebUSB) needs an ESP32-S3 (also S2 / P4) — not the
  classic ESP32. `sdkconfig.defaults` pins `esp32s3` and enables the TinyUSB
  vendor class.
- WebUSB / Web Serial are Chromium-only and need a secure context (`https`,
  `http://localhost`, or `file://`).
