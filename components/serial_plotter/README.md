# Serial Plotter

A self-contained browser tool for reading columnar serial data and plotting it
efficiently — modeled on [esp-cpp/uart_serial_plotter](https://github.com/esp-cpp/uart_serial_plotter),
but running entirely in a Chromium-based browser over the Web Serial API. No
install, no CDN, no network access.

- **Hosted:** <https://esp-cpp.github.io/espp/apps/serial_plotter.html>
- **Offline:** open `web/serial_plotter.html` directly via a `file://` URL.

> This component currently ships the webapp only. A firmware-side binary
> **telemetry** transport (a `stream_frame` / `dispatcher` module for
> higher-bandwidth, typed channels) is a planned follow-up; the same webapp will
> gain a WebUSB transport that feeds the same plot.

## Features

- **Automatic parsing.** The delimiter (comma / tab / semicolon / whitespace) is
  detected per line. A line of non-numeric labels is treated as a **header**; the
  first numeric row of matching width confirms it and becomes the schema. Rows
  that do not match the schema (wrong column count, non-numeric, or log lines)
  are **discarded**. A header only binds the row that follows it, so ordinary
  ESP-IDF log lines never hijack the schema.
- **Re-evaluates on a new header.** When a different header arrives mid-stream,
  the next matching row starts a fresh dataset — just like `uart_serial_plotter`.
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

## Requirements

Web Serial is available only in Chromium-based browsers (Chrome, Edge, Opera) and
needs a secure context — it works from `https`, `http://localhost`, or `file://`.
In an unsupported browser the app still loads and can **Load CSV** for viewing.

## Third-party

Plotting uses **uPlot** (`web/uPlot.iife.min.js`), MIT-licensed, pinned to
v1.6.31 — <https://github.com/leeoniya/uPlot>. It is vendored as a sibling `.js`
file (the docs workflow ships `web/*.html` and `web/*.js`); uPlot's small CSS is
inlined into `serial_plotter.html`. Everything else is dependency-free.
