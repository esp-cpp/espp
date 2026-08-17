# USB CDC + ODrive ASCII Example

This example demonstrates the `espp::UsbCdc` native-USB CDC-ACM transport wired to
the transport-agnostic `espp::OdriveAscii` protocol server. The device enumerates
as a dedicated USB serial port with an ODrive-like VID/PID (0x1209 / 0x0d32),
separate from the log console which stays on the USB-Serial-JTAG peripheral.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [USB CDC + ODrive ASCII Example](#usb-cdc--odrive-ascii-example)
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

## Build

```sh
cd components/usb_device/example
idf.py set-target esp32s3
idf.py build
```

## Flash and Monitor

Flash / monitor over the USB-Serial-JTAG (or UART) console, which is kept separate
from the native USB CDC interface:

```sh
idf.py flash monitor
```

The native USB-OTG connector will appear on the host as a new serial port with
manufacturer "espp" and product "espp ODrive ASCII".

## Usage

Open the native USB serial port and send ODrive ASCII commands, e.g. from Python:

```python
import serial
# The port that enumerated with VID 0x1209 / PID 0x0d32
ser = serial.Serial('/dev/tty.usbmodemXXXX', 115200, timeout=0.5)

ser.write(b'r axis0.encoder.pos_estimate\n'); print(ser.readline())
ser.write(b'w axis0.controller.input_pos 12.34\n'); print(ser.readline())
ser.write(b'p 0 1.0 0.5 0.1\n'); print(ser.readline())
ser.write(b'f 0\n'); print(ser.readline())
```

## How it works

- `espp::UsbCdc` installs the TinyUSB driver and a single CDC-ACM interface with
  the configured VID/PID/strings.
- Its receive callback feeds incoming bytes to `espp::OdriveAscii::process_bytes()`.
- The returned response bytes are written back out over `espp::UsbCdc::write()`.
- The log console remains on the USB-Serial-JTAG peripheral (see
  `sdkconfig.defaults.esp32s3`).
