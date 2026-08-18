# ODrive WebUSB browser tools

This directory hosts two single-file, fully offline WebUSB tools for the espp
ODrive firmware. Both talk to the firmware's dedicated USB **vendor** interface
(`bInterfaceClass = 0xFF`, one bulk IN + one bulk OUT), discover the interface
and endpoints from the descriptors at runtime, and have **no external
dependencies, no CDN, and no network fetches** (they work from a `file://` URL).

| File | Protocol on the vendor interface | Use it for |
| --- | --- | --- |
| [`odrive_webusb_console.html`](./odrive_webusb_console.html) | ODrive **ASCII** text stream | A terminal-style console (raw commands, `r`/`w`/`p`/`v`/`c`/`t`/`es`/`f`). |
| [`odrive_control_panel.html`](./odrive_control_panel.html) | ODrive **native** (legacy Fibre endpoint) **binary** protocol | A driverless-browser **control panel** (property tree, live plots, inline edit) — a replacement for the ODrive Web GUI, for our boards. |

Pick the file that matches the protocol your firmware exposes on the vendor
interface. **The two are not interchangeable** — one carries newline-terminated
ASCII, the other carries packet-based binary Fibre endpoints.

> A third, unrelated tool lives alongside these:
> [`hid_visualizer.html`](./hid_visualizer.html) uses **WebHID** (not WebUSB) to
> read and visualize HID input reports from a USB HID gamepad. See
> [`README_hid.md`](./README_hid.md).

The rest of this document describes the ASCII console; the native control panel
is documented in its own section [at the end](#native-protocol-control-panel-odrive_control_panelhtml).

---

# ODrive ASCII WebUSB Console

A single-file, fully offline browser console for the
[`espp::OdriveAscii`](../include/odrive_ascii.hpp) protocol that talks to the
firmware over **WebUSB** instead of a serial port.

`odrive_webusb_console.html` is pure HTML/CSS/JavaScript with **no external
dependencies, no CDN, and no network fetches** — everything is inline, so it
works entirely offline. It is the WebUSB sibling of
[`odrive_console.html`](./odrive_console.html) (which uses Web Serial); the UI
and command set are identical, only the transport differs.

## What it talks to

Unlike the Web Serial console, this page does **not** open a CDC serial / COM
port. It talks to the firmware's dedicated USB **vendor** interface:

- `bInterfaceClass = 0xFF` (vendor-specific), with one **bulk IN** and one
  **bulk OUT** endpoint.
- That endpoint pair carries the **raw ODrive ASCII byte stream**: the console
  `transferOut`s newline-terminated command lines and `transferIn`s
  newline-terminated responses, with no extra framing.
- The device's default **VID/PID is `0x1209` / `0x0d32`**.

The console **discovers the interface and endpoints at runtime** from the USB
descriptors — interface and endpoint numbers are never hardcoded. After
`navigator.usb.requestDevice`, it calls `open()`, `selectConfiguration(1)`, then
iterates `device.configuration.interfaces[*].alternates[*]` to find the
alternate whose `interfaceClass === 0xFF`, `claimInterface()`s it, and scans its
`endpoints` for the `bulk`/`in` and `bulk`/`out` entries to get the endpoint
numbers used for `transferIn(epIn, len)` / `transferOut(epOut, data)`.

## How to run

WebUSB only works in a **Chromium-based browser** (Chrome, Edge, Opera, Brave —
Firefox and Safari do not implement it) and only in a secure context. For
WebUSB, `https://`, `http://localhost`, **and `file://` all count as secure**,
so you can:

- **Open the file directly** — double-click `odrive_webusb_console.html` or drag
  it into a tab (`file://`). No web server required.
- **Serve it locally**:

  ```sh
  cd components/odrive_ascii/web
  python3 -m http.server 8000
  # then open http://localhost:8000/odrive_webusb_console.html
  ```

- **Use the docs-hosted copy** — this page is also intended to be published on
  the espp GitHub Pages docs (`https://`, a secure context) and linked from the
  `odrive_ascii` documentation, so it can be launched straight from the browser.

Then:

1. Click **Connect** and pick the ODrive vendor device from the WebUSB chooser.
   By default the chooser is filtered to VID `0x1209` / PID `0x0d32`; tick
   **Any device** first to list every USB device instead.
2. Use the quick-control panels or the raw command box to talk to the firmware.
   The status line shows the discovered interface number and the bulk IN/OUT
   endpoint numbers and packet sizes.

**Windows note:** on Windows the device must bind the **WinUSB** driver for
WebUSB to claim the vendor interface. The firmware's **MS-OS-2.0 descriptor**
handles this automatically, so no manual driver install (e.g. Zadig) should be
needed.

## Features

- **Connect / Disconnect** with a default VID/PID filter plus an **Any device**
  option, and a live connection status indicator that reports the discovered
  interface + endpoints.
- **Timestamped log / scrollback** with TX and RX lines in distinct colors,
  autoscroll with pause-on-scroll-up, an "Echo TX" toggle, and a clear button.
- **Raw command input**: type a line and press Enter (a `\n` is appended) or
  click Send. Up/Down arrows recall command history.
- **Read** panel (`r <path>`) with a free-text path field plus a dropdown of
  common ODrive paths.
- **Write** panel (`w <path> <value>`).
- **Setpoint panels** for `p`, `v`, `c`, `t`, and `es` with axis + value fields
  (optional feed-forward args are only appended when filled in).
- **Feedback polling**: read `f <axis>` once, or poll continuously at a
  configurable rate. The parsed `<pos> <vel>` is shown as live metrics and drawn
  on a lightweight inline `<canvas>` sparkline (no plotting libraries).
- **Theme-aware** (respects `prefers-color-scheme`, styled for light and dark),
  responsive, no horizontal body scroll.
- Robust RX handling: a persistent read loop over `transferIn` that tolerates
  endpoint stalls (clears the halt), `babble`, partial lines, and device unplug
  (including the `navigator.usb` `disconnect` event) without throwing uncaught
  errors.

## Command reference

Send `\n`-terminated lines; responses are `\n`-terminated.

| Command | Meaning | Response |
| --- | --- | --- |
| `r <path>` | Read a property | `<value>\n` |
| `w <path> <value>` | Write a property | (none), or `OK\n` / `ERR: ...\n` |
| `p <axis> <pos> [vel_ff [torque_ff]]` | Position setpoint | (none) |
| `v <axis> <vel> [torque_ff]` | Velocity setpoint | (none) |
| `c <axis> <torque_nm>` | Torque / current command | (none) |
| `t <axis> <goal_pos_turns>` | Trajectory goal (turns) | (none) |
| `f <axis>` | Feedback request | `<pos> <vel>\n` |
| `es <axis> <abs_pos_turns>` | Set encoder absolute position (turns) | (none) |
| `help` | Print usage | usage text |

By default `espp::OdriveAscii` matches the ODrive protocol and is **silent** on
writes and setpoints (`w`/`p`/`v`/`c`/`t`/`es`) — only `r`/`f`/`help` respond.
Firmware built with `Config::acknowledge_commands = true` will additionally reply
`OK\n` on success; errors are always reported. The console handles either mode.

## WebUSB vs Web Serial

| | WebUSB (this console) | Web Serial (`odrive_console.html`) |
| --- | --- | --- |
| API | `navigator.usb` | `navigator.serial` |
| Device interface | dedicated USB **vendor** interface (`0xFF`) with a bulk IN/OUT endpoint pair | CDC serial (USB-Serial-JTAG / CDC ACM) |
| OS view | no COM/tty port; the browser claims the raw interface directly | enumerates as a COM/tty serial port |
| Secure context | `https://`, `http://localhost`, **and** `file://` | `https://` and `http://localhost` (also `file://` in Chromium) |
| Windows driver | must bind **WinUSB** (handled by the firmware's MS-OS-2.0 descriptor) | uses the OS's built-in serial/CDC driver |
| Launch from a web page | yes — a landing page can call `requestDevice()` and auto-launch the console | user must pick a serial port |
| Interleaving | dedicated interface: only ODrive ASCII bytes | if the CDC link also carries logger output, those bytes interleave with responses |

In short: **WebUSB uses a dedicated vendor interface** (no serial port, and it
can be auto-launched from a hosted landing page), while **Web Serial uses the
CDC serial port** the OS already enumerates. Both carry the same ODrive ASCII
byte stream and expose the same console UI; pick whichever transport your
firmware build exposes.

## Hosting

This page is intended to also be hosted on the espp GitHub Pages docs (an
`https://` secure context) and linked from the `odrive_ascii` documentation, so
users can open the WebUSB console directly from the docs without downloading
anything.

---

# Native-protocol control panel (`odrive_control_panel.html`)

A single-file, fully offline **control panel** — the driverless-browser
replacement for the ODrive Web GUI, but for our boards. Unlike the ASCII console
above, it speaks the ODrive **native (legacy Fibre endpoint) binary protocol**
over the same USB **vendor** interface: it downloads the device's object model
from endpoint 0, renders it as a live property tree, and does typed get/set by
endpoint id. Same runtime interface/endpoint discovery, same "no CDN / no
dependencies / works from `file://`" guarantees.

## The native protocol, in the browser

The authoritative wire spec is
[`components/odrive_native/PROTOCOL.md`](../../odrive_native/PROTOCOL.md); the
panel implements the client side of it in plain JavaScript:

- **Packets, not a stream.** Over WebUSB each `transferOut` sends exactly one
  request packet and each `transferIn` returns exactly one response packet —
  **there is no UART stream framing** (USB provides the reliability the stream
  framing otherwise adds). Every request is
  `[seq u16][endpoint u16 (| 0x8000 to expect a response)][output_len u16][payload][trailer u16]`,
  little-endian; a response is `[seq | 0x8000 u16][data…]`.
- **CRC-16** (poly `0x3d65`, non-reflected, MSB-first). The endpoint **trailer
  canary** is `PROTOCOL_VERSION` (`1`) for endpoint 0, else the **`json_crc`**,
  which is `crc16(json_bytes, init=1)` — **not** the `0x1337` stream-framing init.
  The panel self-tests the CRC against the golden vector
  `crc16("123456789", init=0x1337) === 0xaa01` on load (see below).
- **Endpoint 0 = the JSON object model.** The panel reads it in ≤512-byte chunks
  (payload = `u32 LE` offset; loop until an empty response), assembles the
  compact JSON tree (`{"name","id","type","access"}` leaves and
  `{"name","type":"object","members":[…]}` nodes) and computes `json_crc`.
- **Typed get/set by id**, little-endian codecs for
  `bool`/`int8`/`uint8`/`int16`/`uint16`/`int32`/`uint32`/`int64`/`uint64`/`float`
  (64-bit via `BigInt`). Reads send `[seq][id|0x8000][size][][json_crc]` and
  decode the response; writes send `[seq][id|0x8000][0][value…][json_crc]` and
  read the value back.

## Features

- **Property tree**: after connecting, endpoint 0 is downloaded and rendered as a
  collapsible tree. Filter by path, expand/collapse, and reload. Each readable
  leaf has a **↻ read-once** button; tick **W** to watch (poll & show the live
  value) and **P** to plot it. Click a `rw` value to **edit inline** (Enter does a
  typed write then reads back, Esc cancels).
- **Live time-series plot**: a real multi-signal `<canvas>` plot (no plotting
  libraries) with a rolling time window, autoscale, y/time gridlines and labels,
  a legend showing each signal's latest value, and pause/clear.
- **Quick controls**: axis-scoped buttons that write `input_pos` / `input_vel` /
  `input_torque` and set `requested_state`, plus a generic **read/write property
  by path** panel. Paths resolve to endpoint ids from the downloaded tree, so a
  path that doesn't exist on this firmware is skipped with a log note.
- **Robust request/response loop**: transactions are serialized on a promise
  chain so every `transferIn` pairs with its `transferOut`; every request sets the
  response bit (writes return a 2-byte ack) so the link stays 1:1. Endpoint
  stalls are cleared (`clearHalt`), and a per-transaction timeout + `clearHalt` is
  the safety net for a silent/unplugged device — no uncaught throws. WebUSB is
  feature-detected; the `navigator.usb` `disconnect` event is handled.
- **Theme-aware** (light/dark via `prefers-color-scheme` and an explicit
  `data-theme` override), responsive, no horizontal body scroll.

## Self-test / verification

On load the panel prints a CRC-16 self-test to the browser console **and** the
in-page log:

```
CRC16 self-test: crc16("123456789", init=0x1337) = 0xaa01 PASS ✓ (expected 0xaa01)
```

plus a codec round-trip check. The wire logic is additionally validated offline:
the CRC matches the golden vector, and a mock server mirroring
`espp::detail::OdriveNativeCore` confirms endpoint-0 chunked download +
`json_crc`, typed read, typed write + read-back, canary rejection, and read-only
enforcement all round-trip correctly.

The control panel has also been **hardware-validated end-to-end**: it was
connected to a real board over WebUSB, downloaded and parsed the endpoint-0 JSON
descriptor, and performed live typed reads/writes and polled plotting against the
device.

## How to run

Identical to the ASCII console: open `odrive_control_panel.html` directly
(`file://`), serve it locally (`python3 -m http.server`), or use the
docs-hosted copy. WebUSB requires a **Chromium-based browser** over `https://`,
`http://localhost`, or `file://`. On Windows the vendor interface must bind
**WinUSB** (handled by the firmware's MS-OS-2.0 descriptor).
