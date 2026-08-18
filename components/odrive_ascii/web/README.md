# ODrive ASCII Web Serial Console

A single-file, fully offline browser console for the
[`espp::OdriveAscii`](../include/odrive_ascii.hpp) protocol.

`odrive_console.html` is pure HTML/CSS/JavaScript with **no external
dependencies, no CDN, and no network fetches** — everything is inline, so it
works entirely offline. It talks to a board using the browser's
[Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API).

The ODrive ASCII protocol runs over the board's serial / USB-CDC link: the
console sends newline-terminated ASCII commands and the firmware
(`espp::OdriveAscii::process_bytes`) returns newline-terminated ASCII responses.
Received bytes may arrive in arbitrary chunks, so the console buffers RX and
splits on `\n`.

## How to run

Web Serial only works in a **secure context** and only in **Chromium-based
browsers** (Chrome, Edge, Opera, Brave — Firefox and Safari do not implement it).

In Chromium, a `file://` URL **is** a secure context, so in most setups you can
just open `odrive_console.html` directly (double-click it, or drag it into a
tab) and it will work — no web server required. Then:

1. Click **Connect** and pick the board's USB-Serial-JTAG / CDC port from the
   chooser. Choose a baud rate if you like (default `115200`; it is largely
   ignored for native USB-CDC but matters for real UART bridges).
2. Use the quick-control panels or the raw command box to talk to the firmware.

If your browser build or an enterprise policy blocks Web Serial on `file://`
(the **Connect** button does nothing / `navigator.serial` is undefined), or you
want to serve the console to another machine, host it over `http://localhost`
or `https://` instead:

```sh
cd components/odrive_ascii/web
python3 -m http.server 8000
# then open http://localhost:8000/odrive_console.html
```

(`http://localhost` and `https://` are secure contexts; a plain `http://` URL to
a remote host is not, so network sharing needs `https`.)

## Features

- **Connect / Disconnect** with a selectable baud rate and a live connection
  status indicator.
- **Timestamped log / scrollback** with TX and RX lines shown in distinct
  colors, autoscroll with pause-on-scroll-up, an "Echo TX" toggle, and a clear
  button.
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
- Robust RX handling: a persistent, line-buffered read loop that tolerates
  partial lines and device unplug / stream-end without throwing uncaught errors.

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

## WebSerial vs WebUSB

This console uses **Web Serial**, which works today over the board's built-in
USB-Serial-JTAG CDC port (and any TinyUSB CDC ACM interface) — the same serial
device your OS enumerates as a COM/tty port. No special driver or vendor
interface is required.

**WebUSB** is a different API that would require the firmware to expose a
dedicated USB *vendor* interface (a raw bulk endpoint pair) rather than a CDC
serial class. That is a separate, future path and is not needed for the ASCII
protocol; it is the direction one would take to implement ODrive's native Fibre
protocol / `odrivetool` auto-discovery.

Note: if the same CDC link also carries other console/log output (e.g. the
firmware's logger), those bytes will be **interleaved** with ODrive ASCII
responses in the RX log. The console tolerates unrelated lines, but for clean
operation prefer a dedicated CDC/UART for the ODrive ASCII protocol.
