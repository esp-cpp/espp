# USB Host Example

This example uses `espp::UsbHost` to drive the ESP32-S3 USB-OTG peripheral as a
**USB host**. It enumerates attached USB **HID** devices (mice, keyboards,
gamepads, or vendor HID devices such as an espp `WdiUsbPeripheral`), logs each
connected device's identity and report-descriptor length, and hex-dumps every
Input report the device sends.

## How it works

- Constructs an `espp::UsbHost` with `on_device_connected` /
  `on_device_disconnected` callbacks.
- On connect, reads the device `info()` (VID/PID + strings) and `params()`
  (interface / protocol), fetches the HID `report_descriptor()`, and installs a
  per-device input callback that logs each report.
- `initialize()` installs the USB Host library + HID class driver and starts the
  event tasks; the device callbacks then fire as devices are plugged / unplugged.

## Hardware / build notes

- USB-OTG **host** mode requires an **ESP32-S2 / -S3 / -P4**, and the board must
  be able to source **VBUS** to the attached device (a board with a USB-A host
  port / VBUS switch, or a self-powered hub).
- The native USB-OTG port is used for the host role, so the console runs on
  **UART0** (USB-Serial-JTAG shares the PHY on the ESP32-S3). Monitor over a UART
  adapter.
- The USB Host library (`usb`) and `usb_host_hid` come from the ESP Component
  Registry, so build this example with the **component manager enabled** (the
  default `idf.py build`), not the manager-off flow used by the device-side USB
  examples.

## Use it with the WDI USB device

Flash the `wdi` component's `usb_example` onto a second ESP32-S3 (it enumerates
as a WDI HID device) and connect it to the host running this example — you will
see the WDI Control / keepalive Input reports arrive in the log. This is the
basis for the forthcoming WDI **host** (wheelchair) role.

## Build and flash

```
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```
