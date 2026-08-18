# espp Board Console &amp; ESP Flasher (Web Serial)

`board_console.html` is a single-file, general-purpose browser tool for **any**
espp / ESP board:

- **Serial monitor** — connect to a board's serial port with the
  [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API),
  watch its output live, send commands (configurable CR/LF/none line ending,
  command history), autoscroll / pause / clear / save-log-to-file, plus
  **Reset board** and **Enter bootloader** buttons that toggle the DTR/RTS
  control signals (RTS = EN/reset, DTR = GPIO0/boot).
- **ESP flasher** — flash your own `.bin` files to the connected chip using
  Espressif's official [esptool-js](https://github.com/espressif/esptool-js).
  Add one row per binary (file picker + hex flash **offset**), or drop a single
  merged image at `0x0`. Options: erase-entire-flash, flashing baud rate. The
  flasher resets the board into its ROM bootloader, detects the chip (name /
  MAC / flash size), writes each file with a per-file progress bar, then
  hard-resets the board.

It is theme-aware (light / dark, with a manual toggle), responsive, and makes
**no third-party network requests** — the only external resource is the
same-origin vendored `esptool-bundle.js`.

## Running it

Web Serial only works in a **Chromium-based browser** (Chrome, Edge, Opera,
Brave — Firefox and Safari do not implement it) served from a secure context:
`https://`, `http://localhost`, or a `file://` URL.

- **Hosted copy:** <https://esp-cpp.github.io/espp/apps/board_console.html>.
  This is also the default WebUSB landing page advertised by the espp
  `usb_device` component (open it, then click **Connect**).
- **Locally:** serve this directory over http and open the page, e.g.
  ```sh
  cd components/usb_device/web
  python3 -m http.server 8000
  # then browse to http://localhost:8000/board_console.html
  ```
  (Both `board_console.html` and `esptool-bundle.js` must be served from the
  same origin, because the page imports the bundle as an ES module.)

### Which port to connect to

Connect to the board's **USB-Serial-JTAG** or **UART** port — *not* a USB CDC
port created by the firmware itself (e.g. the espp `usb_device` CDC function).
The flasher drives the ROM serial bootloader over that port by pulsing the
reset / GPIO0 lines, which a firmware-created CDC port cannot do.

### Flash offset guidance

Offsets are entered in hex. Typical layout:

| Region                     | Offset                                                        |
| -------------------------- | ------------------------------------------------------------- |
| Second-stage bootloader    | `0x0` on ESP32-S3 / -C3 / -C6 / -H2; `0x1000` on ESP32 / -S2  |
| Partition table            | `0x8000`                                                      |
| Application (factory app)  | `0x10000`                                                     |

A single **merged** image (e.g. produced by `esptool.py merge_bin`) goes in one
row at `0x0`. The offsets ESP-IDF actually uses for a given project are printed
by `idf.py build` (see `flash_args` / the "flash" output).

## esptool-js attribution

The flashing feature is powered by **esptool-js**
(<https://github.com/espressif/esptool-js>), © Espressif Systems, licensed under
the **Apache License 2.0**. To keep this app free of any runtime third-party CDN
dependency, esptool-js is **vendored same-origin**: the published bundled build
is committed here verbatim as `esptool-bundle.js` and imported by
`board_console.html` via `import { ESPLoader, Transport } from './esptool-bundle.js'`.

- Pinned version: **esptool-js 0.5.7**
- Source of the bundle:
  `https://cdn.jsdelivr.net/npm/esptool-js@0.5.7/bundle.js`

To update it, re-download that pinned URL (bump the version) and replace
`esptool-bundle.js`; no change to `board_console.html` is required as long as
the bundle keeps exporting `ESPLoader` and `Transport`.
