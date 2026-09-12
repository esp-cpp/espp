# Switch Pro Controller (NS1) Component

[![Badge](https://components.espressif.com/components/espp/switch_pro/badge.svg)](https://components.espressif.com/components/espp/switch_pro)

`espp::SwitchPro` is a **Nintendo Switch Pro controller (NS1) USB emulation
protocol engine**. It implements the Switch Pro controller's USB HID handshake
and input-report protocol so an ESP32-S3 (or other native-USB ESP) can present
itself to a Nintendo Switch as a Pro Controller.

It is **transport-light**: the class owns the controller state and the
request/response state machine but performs no USB I/O itself. Drive it from a
USB HID interface — the espp [`usb_device`](../usb_device) component's HID
function is the intended pairing (see the [example](./example)):

- feed every host **OUTPUT** report (the handshake: `0x80` init, `0x01`
  output/subcommand, `0x10` rumble) to `on_hid_report()`,
- send the reply it returns (and the periodic standard input report from
  `get_input_report()`) back as HID **INPUT** reports.

The HID **report descriptor**, input-report packing, and the SPI-ROM
calibration/config blobs come from the espp [`hid-rp`](../hid-rp) component
(`switch_pro_descriptor()`, `SwitchProGamepadInputReport`).

> ### ⚠️ Emulation only
> A real Switch only binds a device that advertises Nintendo's Pro Controller USB
> **VID/PID `0x057E` / `0x2009`** and identity strings (exposed here as
> `espp::SwitchPro::vid` / `pid` / `manufacturer_name` / `product_name`). Use
> these to emulate / test against a Switch **you own**. Do not ship a product that
> impersonates Nintendo hardware.

## Features

- Full USB **handshake** state machine: the `0x80`/`0x81` init exchange
  (device-info, handshake echo, baud rate, enable-USB-HID) and the `0x01`
  subcommand protocol (request device info, SPI flash read of calibration/config,
  set input mode, set player lights, enable IMU/vibration, trigger-button elapsed
  times, NFC/IR config stubs, shipment, …).
- Emulated **SPI flash** (factory calibration `0x60` + user calibration `0x80`
  banks) with a randomized serial number and the ESP's factory MAC.
- Thread-safe **input** updates via `update_input_report()` using the `hid-rp`
  setters (buttons, D-pad, sticks), plus a 4.96 ms input-report counter timer.
- **No exceptions**; idiomatic espp (`espp::Logger` via `BaseComponent`).

## Protocol / handshake

The Switch drives initialization over the interrupt-OUT endpoint; the controller
replies on the interrupt-IN endpoint. Report ids: host → device `0x80` (init),
`0x01` (output + subcommand), `0x10` (rumble); device → host `0x81` (init reply),
`0x21` (subcommand reply), `0x30` (standard full input report). The
implementation follows the community reverse-engineering references
([dekuNukem](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering),
[nxbt](https://github.com/Brikwerk/nxbt)).

Typical flow: on attach the controller sends its `0x81` device-info report to kick
things off; the Switch then issues subcommands (`0x02` device info, `0x10` SPI
reads for calibration, `0x03` set report mode, `0x30` player lights, `0x40` IMU,
`0x48` vibration, …), each answered with a `0x21` subcommand reply; once it sends
`enable USB HID` the controller streams `0x30` standard input reports.

## API

Key class: `espp::SwitchPro` (`switch_pro.hpp`)

- `get_report_descriptor()` — the HID report descriptor bytes.
- `on_attach()` → `optional<ReportData>` — the initial report to send on mount.
- `on_hid_report(report_id, data, len)` → `optional<ReportData>` — handle a host
  OUTPUT report; returns the INPUT report to send back (`{report_id, bytes}`).
- `get_input_report()` — the current standard (`0x30`) input report bytes.
- `update_input_report(fn)` — thread-safely set buttons / sticks via the `hid-rp`
  report setters.
- `is_ready()` — whether the host has enabled input reports.
- `set_battery_level()`, `set_trigger_elapsed_times()`.
- Identity constants: `vid`, `pid`, `bcd_device`, `bcd_usb`, `manufacturer_name`,
  `product_name`, `input_report_id()`.

## Example

The [example](./example) emulates an NS1 Switch Pro controller over USB using
`espp::usb_device`'s HID function: it advertises the Switch Pro descriptor with an
interrupt-OUT endpoint, funnels received OUTPUT reports to `on_hid_report()`, and
streams input reports (cycling A/B/X/Y and sweeping the left stick) once the host
is ready. Build for the ESP32-S3, flash, and plug the native USB port into a
Switch (or a PC with a Switch-Pro-aware driver / tester).

```sh
cd example
idf.py set-target esp32s3 build flash monitor   # console is on UART0
```

The example's `sdkconfig.defaults` enables the TinyUSB HID class
(`CONFIG_TINYUSB_HID_COUNT=1`) with a 64-byte HID endpoint buffer
(`CONFIG_TINYUSB_HID_BUFSIZE=64`, required for the 64-byte Switch reports) and
keeps the console on UART0 (the native USB port is taken by the HID interface;
see the `usb_device` docs on the shared ESP32-S3 USB PHY).

## Requirements

- An ESP with the native USB-OTG peripheral (ESP32-S3 / S2 / P4).
- espp `usb_device` (or another USB HID transport), `hid-rp`, `timer`.
