# USB Host Component

[![Badge](https://components.espressif.com/components/espp/usb_host/badge.svg)](https://components.espressif.com/components/espp/usb_host)

`espp::UsbHost` is the host-side counterpart to `espp::UsbDevice`. It drives the
ESP32-S2 / -S3 / -P4 USB-OTG peripheral as a **USB host**, enumerates attached
devices, and exposes the **HID** class devices it finds — mice, keyboards,
gamepads, and vendor-specific HID devices (for example another ESP running
`espp::UsbDevice` as a HID device, such as an `espp::WdiUsbPeripheral`).

It is a thin, idiomatic espp wrapper over the ESP-IDF USB Host library (`usb`)
and the `usb_host_hid` class driver: it owns the whole host lifecycle (install
the host library + HID driver, run the event tasks, open interfaces, teardown)
and marshals the driver's C callbacks into per-device `std::function`s. Like the
rest of espp it does not throw and reports failures via `std::error_code`.

## Features

- Installs / uninstalls the USB Host library and the HID class driver and runs
  their event-handling tasks.
- Device **connect / disconnect** callbacks, with an optional filter predicate so
  you only open the devices you care about (by VID/PID, interface, etc.).
- Per-device **Input report** callback (device→host), delivering the raw report
  bytes (report ID in byte 0 for report-ID'd descriptors — symmetric with how
  `espp::UsbDevice`'s HID receive callback delivers OUT reports).
- Send **Output reports** (host→device) and issue the HID class control requests
  (Get/Set Report, Get/Set Idle, Set Protocol).
- Read a connected device's **HID report descriptor**.

Report directions are named from the connected **device's** point of view, as in
the USB HID spec: an *Input* report is device→host, an *Output* report is
host→device. This mirrors `espp::UsbDevice` exactly, so the two ends of a link
(e.g. the device and host roles of the `wdi` component) line up.

## Requirements / caveats

- USB-OTG **host** mode is only available on the **ESP32-S2, -S3 and -P4**.
- Only **one** `espp::UsbHost` may exist at a time (the USB Host library and HID
  class driver are global singletons). It cannot coexist with `espp::UsbDevice`
  (they both claim the USB-OTG peripheral).
- The board must be able to source **VBUS** to the attached device — a board with
  a USB-A host port / VBUS switch, or a self-powered hub. `UsbHost` does not
  manage board power.
- On the ESP32-S3 the USB-Serial-JTAG shares the USB-OTG PHY, so when the host
  role is active the **console must run on UART0** (see the example's
  `sdkconfig.defaults`).
- The USB Host library (`usb`) and `usb_host_hid` come from the ESP Component
  Registry via the IDF component manager. On ESP-IDF ≥ 6.0 `usb_host_hid`
  declares its `usb` dependency only through the manager, so build the example
  with the component manager **on** (the default) rather than the manager-off
  flow used by the device-side USB examples.

## Threading model

The HID class driver delivers its events on its own background task — and that
same task is what completes the driver's *synchronous* control transfers
(Set/Get Report, Set Protocol, …). A control transfer issued *from* that task can
therefore never complete (it times out). `UsbHost` handles this the way the
ESP-IDF HID host example does, but internally: the driver task only **enqueues**
events (copying each Input report out of the driver's buffer, which must happen
inside the callback), and a dedicated **dispatch task** owned by `UsbHost`
opens/starts/closes devices and invokes every user callback. Consequences:

- It is safe to call `send_output_report()` and the other `HidDevice` methods
  from inside `on_device_connected` / the input callback (e.g. to answer a
  request/response HID protocol) — they complete normally.
- Events for a device are delivered in order (connected → inputs →
  disconnected), and the connect callback runs *before* the device is started,
  so an input callback installed there sees the very first report.
- `HidDevice` methods may also be called from any application task; each device
  serializes its driver calls internally, so an app-task control transfer can't
  race the device being retired on disconnect.
- Keep callbacks reasonably short: one that blocks delays every later event.
- The queue is bounded (`Config::max_queued_events`); if the consumer falls
  behind, Input reports are dropped (logged) rather than blocking the USB stack.

`HidDevice::info()` / `params()` / `report_descriptor()` are snapshots taken at
connect time, so they remain valid after the device disconnects.

## Example

```cpp
#include "usb_host.hpp"

espp::UsbHost host({
    .on_device_connected =
        [](const std::shared_ptr<espp::UsbHost::HidDevice> &dev) {
          auto info = dev->info();
          printf("connected: %s %s %04x:%04x\n", info.manufacturer.c_str(),
                 info.product.c_str(), info.vid, info.pid);
          dev->set_input_callback([](std::span<const uint8_t> report) {
            // handle a device->host Input report (report[0] is the report id)
          });
        },
    .on_device_disconnected = [](const auto &) { /* ... */ },
    // optional: only open the devices you want
    // .should_open = [](const auto &info, const auto &) { return info.vid == 0x1209; },
});

std::error_code ec;
if (!host.initialize(ec)) { /* handle ec */ }

// later, send an Output report (host->device) -- devices() may be empty, so
// guard it (or keep the shared_ptr handed to on_device_connected and use that):
std::array<uint8_t, 4> payload{...};
if (auto devs = host.devices(); !devs.empty())
  devs.front()->send_output_report(/*report_id*/ 0x02, payload, ec);
```

See `example/` for a full runnable example (esp32s3) that logs every connected
HID device and hex-dumps its Input reports.

## Roadmap

Only the **HID** class driver is wired up today (it covers mice, keyboards,
gamepads and vendor HID devices, and is what the `wdi` host role needs). The
component is structured so other class drivers (CDC-ACM, MSC) can be layered in
later without changing the host-lifecycle model — the same way `espp::UsbDevice`
composes CDC / Vendor / HID functions on the device side.
