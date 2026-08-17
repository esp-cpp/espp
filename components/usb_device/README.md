# USB Device Component

[![Badge](https://components.espressif.com/components/espp/usb_device/badge.svg)](https://components.espressif.com/components/espp/usb_device)

`espp::UsbDevice` is an idiomatic wrapper around ESP-IDF's `esp_tinyusb` managed
component that assembles a **native USB device** from a *set of selectable
functions* on the ESP32-S3 / -S2 / -P4 USB-OTG peripheral, with a **configurable
VID/PID** and manufacturer / product / serial strings.

Today it can enable, in any combination (subject to the endpoint budget):

- A **CDC-ACM** function (virtual serial port).
- A **vendor-specific** function (`bInterfaceClass` 0xFF, one bulk IN + one bulk
  OUT) carrying a raw byte stream, optionally advertising **WebUSB** + **MS OS
  2.0** descriptors so a browser can talk to it driverlessly (and Windows binds
  WinUSB with no driver).

Interface numbers, endpoint addresses and string indices are allocated
*sequentially* as functions are enabled, and the result is checked against the
USB-OTG endpoint budget. Because it uses the native USB-OTG peripheral (not the
built-in USB-Serial-JTAG that carries the ESP console), a device can advertise its
own USB identifiers (e.g. ODrive-like) on a link that is completely separate from
the logging console.

`espp::UsbCdc` is retained as a thin **CDC-only preset** over `espp::UsbDevice`
for back-compatibility.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [USB Device Component](#usb-device-component)
  - [Features](#features)
  - [API](#api)
  - [Enabling the vendor / WebUSB class](#enabling-the-vendor--webusb-class)
  - [Endpoint budget (ESP32-S3 USB-OTG)](#endpoint-budget-esp32-s3-usb-otg)
  - [Extending with HID / MSC](#extending-with-hid--msc)
  - [Example](#example)
  - [Notes](#notes)

<!-- markdown-toc end -->

## Features

- **Composable**: enable a CDC function and/or a vendor/WebUSB function (composite).
- **Vendor-specific interface** (class 0xFF): raw bulk IN + bulk OUT byte stream.
- **WebUSB**: BOS + WebUSB URL + MS OS 2.0 descriptors for driverless browser
  access, with a configurable landing-page URL.
- **Sequential allocation** of interfaces / endpoints / strings with an
  endpoint-budget check (error via `std::error_code` if exceeded).
- **Configurable identity**: VID, PID, manufacturer / product / serial / interface
  strings.
- **Idiomatic espp**: no exceptions; `initialize()` reports failures via
  `std::error_code`.
- **Safe marshaling**: the TinyUSB RX callbacks (TinyUSB task context) are drained
  and delivered to per-function user callbacks; the matching `write_*()` is safe to
  call from within them.

## API

Composite CDC + vendor/WebUSB device (both interfaces carry the same raw stream):

```cpp
espp::UsbDevice::Config cfg;
cfg.vid = 0x1209;  // pid.codes VID (ODrive uses this)
cfg.pid = 0x0d32;  // ODrive-like PID

espp::UsbDevice::CdcFunction cdc;
cdc.on_receive = [&](std::span<const uint8_t> data) { /* serial rx */ };
cfg.cdc = cdc;

espp::UsbDevice::VendorFunction vendor;
vendor.webusb = true;  // advertise WebUSB / MS OS 2.0 descriptors
// vendor.landing_page_url defaults to the espp docs-hosted ODrive WebUSB console,
// without a scheme; vendor.url_scheme selects http (0) or https (1).
vendor.on_receive = [&](std::span<const uint8_t> data) { /* vendor rx */ };
cfg.vendor = vendor;

espp::UsbDevice usb(cfg);
std::error_code ec;
if (!usb.initialize(ec)) { /* handle ec (e.g. endpoint budget exceeded) */ }

uint8_t hello[] = {'h','i','\n'};
usb.write_cdc(hello);
usb.write_vendor(hello);
```

Key methods:

- `bool initialize(std::error_code &ec)` — build descriptors from the enabled
  functions, check the endpoint budget, install the TinyUSB driver.
- `bool write_cdc(...)` / `bool write_vendor(...)` — queue + non-blocking flush on
  the respective interface.
- `void set_cdc_receive_callback(...)` / `void set_vendor_receive_callback(...)`.
- `bool is_cdc_connected() const` / `bool is_vendor_connected() const`.

CDC-only preset (`espp::UsbCdc`, unchanged API): `initialize()`, `write()`,
`set_receive_callback()`, `is_connected()`.

## Enabling the vendor / WebUSB class

The vendor class is gated in `esp_tinyusb` behind a Kconfig option. To use the
vendor function, set in your project's `sdkconfig.defaults`:

```
CONFIG_TINYUSB_CDC_ENABLED=y
CONFIG_TINYUSB_CDC_COUNT=1
CONFIG_TINYUSB_VENDOR_COUNT=1   # THE key enablement: compiles in the vendor class
```

Setting `CONFIG_TINYUSB_VENDOR_COUNT` > 0 makes `esp_tinyusb` define
`CFG_TUD_VENDOR` and compile the TinyUSB vendor class driver. No custom
`tusb_config` is needed — the BOS descriptor and the WebUSB / MS-OS-2.0 vendor
control requests are provided by `espp::UsbDevice` through the standard TinyUSB
weak-callback overrides (`tud_descriptor_bos_cb`, `tud_vendor_control_xfer_cb`,
`tud_vendor_rx_cb`). If the vendor function is requested but `CFG_TUD_VENDOR == 0`,
`initialize()` fails with `std::errc::function_not_supported`.

## Endpoint budget (ESP32-S3 USB-OTG)

The ESP32-S3 / -S2 USB-OTG core is full-speed and, besides EP0, provides roughly
**5 usable data IN endpoints** and **5 usable data OUT endpoints**. Each function
consumes:

| Function          | IN endpoints                                | OUT endpoints                  |
|-------------------|---------------------------------------------|--------------------------------|
| CDC-ACM           | 2 (1 interrupt-IN notif + 1 bulk-IN)        | 1 (bulk-OUT)                   |
| Vendor / WebUSB   | 1 (bulk-IN)                                  | 1 (bulk-OUT)                   |
| HID (future)      | 1 (interrupt-IN)                            | 0 or 1 (optional interrupt-OUT) |
| MSC (future)      | 1 (bulk-IN)                                  | 1 (bulk-OUT)                   |

This is why the device is **selectable** ("not all at once"). Combinations that
fit comfortably: CDC+Vendor (3 IN / 2 OUT, used by the example), CDC+Vendor+HID,
CDC+Vendor+MSC. Enabling CDC+Vendor+HID+MSC reaches 5 IN endpoints — at the hard
limit, not recommended. `initialize()` returns `std::errc::value_too_large` if the
IN or OUT budget is exceeded.

## Extending with HID / MSC

`espp::UsbDevice::Config` reserves `std::optional` slots for `HidFunction` and
`MscFunction` as documented extension points. They are not implemented yet;
enabling one today makes `initialize()` fail with
`std::errc::function_not_supported`. When implemented they slot into the same
sequential allocator: HID appends one interface (report descriptor + report
get/set callbacks) claiming an interrupt-IN endpoint; MSC appends one interface
(SCSI + storage read/write/capacity callbacks) claiming a bulk IN + bulk OUT.

## Example

See `example/` for a full project that wires a **composite CDC + Vendor/WebUSB**
`espp::UsbDevice` to the transport-agnostic `espp::OdriveAscii` protocol server.
Both interfaces feed the same server (RX from either interface → `process_bytes`
→ response written back out the same interface), while the log console stays on
the USB-Serial-JTAG peripheral.

## Notes

- USB-OTG is only available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
- Only one `espp::UsbDevice` / `espp::UsbCdc` instance may exist at a time.
- The receive callbacks run in the TinyUSB device task; keep them short and
  non-blocking.
