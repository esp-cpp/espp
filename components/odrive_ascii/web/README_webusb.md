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
