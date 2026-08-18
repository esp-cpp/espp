# ODrive-compatible USB Device Example (CDC + Vendor/WebUSB + HID)

This example demonstrates a **composite** `espp::UsbDevice` that presents an
ODrive-compatible device with three interfaces, all backed by one simulated motor
state (matching how a real ODrive splits its protocols across interfaces):

- **CDC-ACM serial** → the **ODrive ASCII** protocol (`espp::OdriveAscii`; text,
  for a terminal or the Web Serial console).
- **vendor-specific (class 0xFF, WebUSB)** → the **ODrive native / Fibre binary**
  protocol (`espp::OdriveNative`) — the one `odrivetool` / the `fibre` library
  auto-discover and speak over USB.
- **HID** → an animated **gamepad** input device (built with the `hid-rp`
  component; visualize it with the WebHID `hid_visualizer.html`).

The device enumerates with an ODrive-like VID/PID (0x1209 / 0x0d32), separate from
the log console which stays on the USB-Serial-JTAG peripheral.

Each protocol server is transport-agnostic: the CDC RX callback feeds bytes to
`OdriveAscii::process_bytes()`, the vendor RX callback feeds bytes to
`OdriveNative::process_bytes()`, and each writes its response back out the same
interface. The HID interface periodically pushes gamepad input reports.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ODrive-compatible USB Device Example (CDC + Vendor/WebUSB + HID)](#odrive-compatible-usb-device-example-cdc--vendorwebusb--hid)
  - [Requirements](#requirements)
  - [Build](#build)
  - [Flash and Monitor](#flash-and-monitor)
  - [Usage](#usage)
  - [How it works](#how-it-works)

<!-- markdown-toc end -->

## Requirements

- An ESP32-S3 (or -S2 / -P4) with access to the native USB-OTG pins.
- ESP-IDF installed and available in your shell.
- The IDF component manager is enabled for this example so it can fetch the
  managed `espressif/esp_tinyusb` component.

The example's `sdkconfig.defaults` enables the CDC, vendor, and HID classes:

```
CONFIG_TINYUSB_CDC_ENABLED=y
CONFIG_TINYUSB_CDC_COUNT=1
CONFIG_TINYUSB_VENDOR_COUNT=1
CONFIG_TINYUSB_HID_COUNT=1
```

## Build

```sh
cd components/usb_device/example
idf.py set-target esp32s3
idf.py build
```

## Flash and Monitor

Flash / monitor over the USB-Serial-JTAG (or UART) console, which is kept separate
from the native USB interfaces:

```sh
idf.py flash monitor
```

The native USB-OTG connector will appear on the host as a new composite device:
a serial port (CDC), a vendor interface (WebUSB), and a HID gamepad, with
manufacturer "espp" and product "espp ODrive".

## Usage

**CDC serial (ODrive ASCII):** open the CDC serial port and send ODrive ASCII
commands, e.g. from Python:

```python
import serial
ser = serial.Serial('/dev/tty.usbmodemXXXX', 115200, timeout=0.5)
ser.write(b'r axis0.encoder.pos_estimate\n'); print(ser.readline())
ser.write(b'w axis0.controller.input_pos 12.34\n'); print(ser.readline())
ser.write(b'p 0 1.0 0.5 0.1\n'); print(ser.readline())
ser.write(b'f 0\n'); print(ser.readline())
```

(Writes/setpoints are silent by default — ODrive semantics; only `r`/`f` respond.)

**Vendor / WebUSB (ODrive native / Fibre):** the vendor interface (class 0xFF,
bulk IN + bulk OUT) speaks the ODrive native binary protocol. `odrivetool` / the
reference `fibre` library discover it over USB and read/write the endpoint tree;
or, from a Chromium-based browser, open the native-protocol WebUSB control panel
(`odrive_control_panel.html`) and connect. The BOS/WebUSB descriptors point to a
configurable landing-page URL. See `HARDWARE_TEST.md` and `odrive_usb_probe.py`.

**HID (gamepad):** the device also enumerates as a HID gamepad whose sticks and
buttons the firmware animates. Your OS will see a gamepad; to inspect the raw
input reports in the browser, open the WebHID `hid_visualizer.html` in Chromium
and connect.

## How it works

- `espp::UsbDevice` installs the TinyUSB driver and builds descriptors for the
  enabled CDC + vendor + HID functions, allocating interfaces / endpoints
  sequentially (the S3 USB-OTG endpoint budget fits CDC + vendor + HID).
- The vendor function advertises WebUSB + MS OS 2.0 descriptors so a browser (and
  Windows, via WinUSB) can bind it driverlessly.
- The CDC receive callback feeds bytes to `espp::OdriveAscii::process_bytes()` and
  writes the response back out via `write_cdc()`; the vendor receive callback feeds
  bytes to `espp::OdriveNative::process_bytes()` and writes back via
  `write_vendor()`. Both servers share one simulated motor state.
- The HID report descriptor is built with the `hid-rp` component
  (`espp::GamepadInputReport`); the main loop animates the state and pushes reports
  with `write_hid_report()` when the HID interface is ready.
- The log console remains on the USB-Serial-JTAG peripheral (see
  `sdkconfig.defaults.esp32s3`).
