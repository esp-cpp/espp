# Telemetry — USB example (→ Serial Plotter web app)

Streams synthetic float channels from an ESP32-S3 to the browser **Serial
Plotter** web app over USB, using `espp::Telemetry` (a binary telemetry emitter
carried on the `stream_frame` framing, dispatcher module 3).

The hosted app — <https://esp-cpp.github.io/espp/apps/telemetry.html> —
connects on the **vendor (WebUSB)** interface, reads the channel **schema**, and
plots the live **sample** stream. It is the binary, higher-rate,
device-timestamped counterpart to the app's text/CSV Web Serial transport.

## What it does

- Declares four channels — `sine`, `cosine`, `noise`, `ramp` — as the schema.
- A producer task emits one sample (a `float` per channel) every ~10 ms (100 Hz),
  timestamped with the device clock.
- Exposes the stream over the USB **vendor (WebUSB)** interface; a `Dispatcher`
  routes module-3 frames to the emitter and serves capability discovery so the
  browser **Device Hub** lists this device and links to `telemetry.html`.
- The web app can pause/resume the stream and request a rate (`SET_STREAM`), and
  requests the schema on connect (`GET_SCHEMA`).

`espp::Telemetry` itself is transport-agnostic (the `stream_frame` framing works
over CDC / UART / a socket too); this example streams over WebUSB because that is
what the web app's binary path consumes.

Swap the synthetic generator for your real signals: build a `std::array<float, N>`
in channel order and call `telemetry.emit(...)`.

## Build & run

```sh
idf.py -p /dev/ttyACM0 flash monitor   # target esp32s3 (set in sdkconfig.defaults)
```

Then open the Serial Plotter web app, click **Connect (USB)**, and pick the
"espp Serial Plotter" device. The system console/logs go to the separate
built-in USB-Serial-JTAG.

## Notes

- Native USB (vendor / WebUSB) needs an ESP32-S3 (also S2 / P4) — not the
  classic ESP32. `sdkconfig.defaults` pins `esp32s3` and enables the TinyUSB
  vendor class.
- WebUSB / Web Serial are Chromium-only and need a secure context (`https`,
  `http://localhost`, or `file://`).
