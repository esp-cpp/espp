# WebHID gamepad input visualizer

[`hid_visualizer.html`](./hid_visualizer.html) is a single-file, fully offline
browser tool that reads and visualizes HID **input reports** straight from a USB
HID device — for example the espp `usb_device` HID gamepad example, or any HID
gamepad. Like the sibling WebUSB tools in this directory it has **no external
dependencies, no CDN, and no network fetches** (it works from a `file://` URL).

## What it is

The page uses the browser's **WebHID** API (`navigator.hid`). It does **not**
hard-code the report byte layout. Instead it walks the device's parsed report
descriptor —
`device.collections[*].inputReports[*].items[*]` (each item carries `usages` /
`usageMinimum` / `usageMaximum`, `reportSize`, `reportCount`, `reportId`,
`logicalMinimum` / `logicalMaximum`) — builds a per-`reportId` field model with an
accumulating bit offset, and decodes every incoming `inputreport` event's
`DataView` against that model (little-endian bit unpacking, with sign extension
for fields declared with a negative `logicalMinimum`).

It then visualizes the decoded fields with generic fallbacks:

- **Buttons** (usage page `0x09`) as a grid of live indicators.
- **Generic-Desktop X/Y (`0x30`/`0x31`)** and **Rx/Ry (`0x33`/`0x34`)** paired
  into **2D stick dots**, normalized via each axis's logical min/max.
- Other axes — **Z/Rz/sliders** — as **bars**.
- **Hat switch (`0x39`)** as a **d-pad**.
- Any **unrecognized usages** as generic labeled bars.
- A **raw report hex line** plus a per-byte view, and a **reports/sec** readout.

If the device exposes no decodable input-report fields (empty
`device.collections`), the page falls back to a raw per-byte view. It handles the
`navigator.hid` `disconnect` event and never lets a decode error escape and kill
the stream. It is theme-aware (light/dark, honoring the viewer's `data-theme`
override) and responsive.

## How to run

WebHID requires a **Chromium-based browser** (Chrome, Edge, Opera, Brave —
Firefox and Safari do not implement it) served over `https://`,
`http://localhost`, or a `file://` URL.

1. Open `hid_visualizer.html` directly (`file://`), or serve it locally
   (`python3 -m http.server`), or use a docs-hosted copy.
2. Click **Connect gamepad** and pick the device (default filter is
   VID `0x1209`, the pid.codes VID the espp examples use). Use **Any HID device**
   to show every non-protected device in the chooser.
3. Move the sticks / press buttons and watch the visualization update live.

On load the page prints a **bit-decode self-test** to the browser console and the
in-page log (synthetic report + a hand-built item model → expected values,
including cross-nibble reads, sign extension, axis normalization, and hat
mapping).

## WebHID vs WebUSB

The other tools here ([`odrive_webusb_console.html`](./odrive_webusb_console.html),
[`odrive_control_panel.html`](./odrive_control_panel.html)) use **WebUSB**, which
claims a raw **vendor** interface and speaks a custom protocol over bulk
endpoints. **WebHID** instead rides on top of the OS HID driver: it exposes the
already-parsed report descriptor and cannot claim a device that another driver
has bound.

Two practical differences worth noting:

- `requestDevice()` returns an **array** of devices in WebHID (vs a single device
  in WebUSB).
- **`acceptAllDevices` is a WebUSB-only flag.** For "any HID device" in WebHID you
  pass `{ filters: [] }`, which **is** valid for WebHID (unlike WebUSB, where an
  empty-filter request is invalid).

For security the browser **blocks protected HID collections — keyboards and
pointing devices — but allows gamepads** and other generic devices. That is
exactly why this tool targets the HID gamepad example.
