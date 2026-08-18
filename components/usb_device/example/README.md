# USB Device (CDC + Vendor/WebUSB) + ODrive ASCII Example

This example demonstrates a **composite** `espp::UsbDevice` that exposes both a
**CDC-ACM serial** interface and a **vendor-specific / WebUSB** interface, both
wired to the transport-agnostic `espp::OdriveAscii` protocol server. The device
enumerates with an ODrive-like VID/PID (0x1209 / 0x0d32), separate from the log
console which stays on the USB-Serial-JTAG peripheral.

Both interfaces carry the identical raw ODrive ASCII byte stream: RX from either
interface is fed to `process_bytes()`, and the response is written back out the
same interface.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [USB Device (CDC + Vendor/WebUSB) + ODrive ASCII Example](#usb-device-cdc--vendorwebusb--odrive-ascii-example)
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

The example's `sdkconfig.defaults` enables both the CDC and vendor classes:

```
CONFIG_TINYUSB_CDC_ENABLED=y
CONFIG_TINYUSB_CDC_COUNT=1
CONFIG_TINYUSB_VENDOR_COUNT=1
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
a serial port (CDC) plus a vendor interface (WebUSB), manufacturer "espp",
product "espp ODrive ASCII".

## Usage

Serial: open the CDC serial port and send ODrive ASCII commands, e.g. from Python:

```python
import serial
ser = serial.Serial('/dev/tty.usbmodemXXXX', 115200, timeout=0.5)
ser.write(b'r axis0.encoder.pos_estimate\n'); print(ser.readline())
ser.write(b'w axis0.controller.input_pos 12.34\n'); print(ser.readline())
ser.write(b'p 0 1.0 0.5 0.1\n'); print(ser.readline())
ser.write(b'f 0\n'); print(ser.readline())
```

WebUSB: from a Chromium-based browser, open the WebUSB console and connect to the
vendor interface (class 0xFF, bulk IN + bulk OUT). The same ODrive ASCII commands
work over the vendor byte stream. The BOS/WebUSB descriptors point to a
configurable landing-page URL (default: the espp docs-hosted ODrive WebUSB
console).

## How it works

- `espp::UsbDevice` installs the TinyUSB driver and builds descriptors for the
  enabled CDC + vendor functions, allocating interfaces / endpoints sequentially.
- The vendor function advertises WebUSB + MS OS 2.0 descriptors so a browser (and
  Windows, via WinUSB) can bind it driverlessly.
- Each interface's receive callback feeds incoming bytes to
  `espp::OdriveAscii::process_bytes()` and writes the response back out that same
  interface (`write_cdc()` / `write_vendor()`).
- The log console remains on the USB-Serial-JTAG peripheral (see
  `sdkconfig.defaults.esp32s3`).
