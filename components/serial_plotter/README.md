# Serial Plotter

A self-contained browser tool for reading data and plotting it efficiently —
modeled on [esp-cpp/uart_serial_plotter](https://github.com/esp-cpp/uart_serial_plotter),
but running entirely in a Chromium-based browser. No install, no CDN, no network
access. Two transports feed the same plot:

- **Web Serial (text / CSV)** — auto-parses columnar output (a header line plus
  numeric rows) from any device that prints it.
- **WebUSB (binary telemetry)** — an espp device streams typed float channels
  via `espp::Telemetry` (see `include/telemetry_service.hpp` and the
  [example](example/)) for higher rate and device-accurate timestamps.

- **Hosted:** <https://esp-cpp.github.io/espp/apps/serial_plotter.html>
- **Offline:** open `web/serial_plotter.html` directly via a `file://` URL.

## Screenshots

The demo data below is a Lorenz attractor (`time,x,y,z`) loaded via **Load CSV**.

Time series — X is a chosen column (or arrival time / sample index), Y is the
rest, with the per-series filter bar:

![Serial Plotter — time series](https://github.com/user-attachments/assets/64668e83-8ff8-4ba8-9f97-1be5e1c3ad6d)

2D X–Y — pick any column as X (here `x` vs `z`, the classic Lorenz butterfly):

![Serial Plotter — 2D X-Y](https://github.com/user-attachments/assets/41850b0d-eb5d-4122-b467-9f35e0d33e30)

3D X–Y–Z — orbit / zoom point cloud (small dependency-free canvas renderer):

![Serial Plotter — 3D X-Y-Z](https://github.com/user-attachments/assets/60de3ba6-2cb0-42a4-9e61-fde228b5dc55)

## Features

- **Automatic parsing.** The delimiter (comma / tab / semicolon / whitespace) is
  detected per line. A line of non-numeric labels is treated as a **header**; the
  first numeric row of matching width confirms it and becomes the schema. Rows
  that do not match the schema (wrong column count, non-numeric, or log lines)
  are **discarded**. A header only binds the row that follows it, and ESP-IDF log
  lines (`I (123) tag: …`, ANSI colors included) are rejected outright, so
  ordinary logging never hijacks the schema.
- **Re-evaluates on a changed header.** When a header with different labels or a
  different column count arrives mid-stream, the next matching row starts a fresh
  dataset — just like `uart_serial_plotter`. A repeat of the same header keeps the
  running dataset (so periodic header echoes don't wipe your capture).
- **High point counts.** Samples land in fixed-capacity per-series ring buffers
  (`Float32Array`) and are drawn with [uPlot](https://github.com/leeoniya/uPlot),
  which does the pixel decimation. Redraws are coalesced to one per animation
  frame. The retained-points cap is configurable (default 200k per series).
- **Series filter.** A filter bar shows a colored chip per column: click to
  toggle a series on/off, or type in the name box to plot only the columns /
  tags that match (composes with the manual toggles), plus **All** / **None**.
- **Plot modes.**
  - *Time series* (default) — X is arrival time, the sample index, or a chosen
    first column; Y is every other column.
  - *2D X–Y* — pick which column is X; plot the rest as lines or points.
  - *3D X–Y–Z* — pick three columns and orbit / zoom a point cloud (a small,
    dependency-free canvas renderer).
- **Save / load CSV.** Download the retained data as CSV, or load a saved capture
  (or any matching CSV) to view it offline with no device connected.
- **Serial controls.** Baud selector, pause / resume, clear, and a DTR/RTS device
  reset.
- **Binary telemetry over WebUSB.** Connect with **USB** to an espp device
  running `espp::Telemetry`: the app reads the channel schema and plots the
  device-timestamped sample stream (decoded from the `stream_frame` framing,
  dispatcher module 3) into the same plot. Requests the schema on connect and
  can pause/resume the device stream.

## Requirements

Web Serial and WebUSB are available only in Chromium-based browsers (Chrome,
Edge, Opera) and need a secure context — they work from `https`,
`http://localhost`, or `file://`. In an unsupported browser the app still loads
and can **Load CSV** for viewing. Native USB telemetry needs an ESP32-S3 (also
S2 / P4) device; see [`example/`](example/).

## Third-party

Plotting uses **uPlot** (`web/uPlot.iife.min.js`), MIT-licensed, pinned to
v1.6.31 — <https://github.com/leeoniya/uPlot>. It is vendored as a sibling `.js`
file (the docs workflow ships `web/*.html` and `web/*.js`); uPlot's small CSS is
inlined into `serial_plotter.html`. Everything else is dependency-free.
