# USB CDC Transport Component

[![Badge](https://components.espressif.com/components/espp/usb_device/badge.svg)](https://components.espressif.com/components/espp/usb_device)

`espp::UsbCdc` is a thin, idiomatic wrapper around ESP-IDF's `esp_tinyusb`
managed component that presents a single dedicated **native USB CDC-ACM**
(virtual serial port) interface on the ESP32-S3 / -S2 / -P4 USB-OTG peripheral,
with a **configurable VID/PID** and manufacturer / product / serial strings.

Because the CDC interface uses the native USB-OTG peripheral (not the built-in
USB-Serial-JTAG that carries the ESP console), a device can advertise its own USB
identifiers (e.g. ODrive-like) on a link that is completely separate from the
logging console. This also lays the groundwork for adding a WebUSB *vendor*
interface later.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [USB CDC Transport Component](#usb-cdc-transport-component)
  - [Features](#features)
  - [API](#api)
  - [Example](#example)
  - [Notes](#notes)

<!-- markdown-toc end -->

## Features

- **Native USB**: uses the USB-OTG peripheral via TinyUSB, separate from the log console.
- **Configurable identity**: VID, PID, manufacturer / product / serial / interface strings.
- **Byte-stream transport**: feed bytes in via a receive callback, send bytes out via `write()`.
- **Idiomatic espp**: no exceptions; `initialize()` reports failures via `std::error_code`.
- **Safe marshaling**: the TinyUSB RX callback (which runs in the TinyUSB task) is drained and
  delivered to the user callback; `write()` is safe to call from within it.

## API

```cpp
espp::UsbCdc::Config cfg;
cfg.vid = 0x1209;               // pid.codes VID (ODrive uses this)
cfg.pid = 0x0d32;               // ODrive-like PID
cfg.manufacturer = "espp";
cfg.product = "espp USB CDC";
cfg.serial_number = "0001";
cfg.on_receive = [](std::span<const uint8_t> data) { /* handle rx */ };

espp::UsbCdc usb(cfg);
std::error_code ec;
if (!usb.initialize(ec)) { /* handle ec */ }

// send bytes
uint8_t hello[] = {'h','i','\n'};
usb.write(hello);

// replace the receive callback at any time
usb.set_receive_callback([](std::span<const uint8_t> data) { /* ... */ });
```

Key methods:

- `bool initialize(std::error_code &ec)` — install the TinyUSB driver + CDC-ACM and set descriptors.
- `bool write(std::span<const uint8_t> data[, std::error_code &ec])` — queue + non-blocking flush.
- `void set_receive_callback(const receive_callback_fn &cb)` — set/replace the RX callback.
- `bool is_initialized() const`, `bool is_connected() const`.

## Example

See `example/` for a full project that wires `espp::UsbCdc` to `espp::OdriveAscii`
so the device shows up as an ODrive-like serial port speaking the ODrive ASCII
protocol, while the log console stays on the USB-Serial-JTAG peripheral.

## Notes

- USB-OTG is only available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
- Enable `CONFIG_TINYUSB_CDC_ENABLED=y` in your project (see the example `sdkconfig.defaults`).
- Only one `espp::UsbCdc` instance may drive a given CDC port.
- The receive callback runs in the TinyUSB device task; keep it short and non-blocking.
