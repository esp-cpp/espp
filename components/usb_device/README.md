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
- A **HID** function (one interrupt IN, optionally one interrupt OUT) carrying an
  application-supplied report descriptor (e.g. a gamepad built with the espp
  `hid-rp` component), with input reports sent via `write_hid_report()`.
- An **X-Input** function that presents the device as a wired **Xbox 360
  controller** (served by a small custom TinyUSB application class driver built
  into this component — no `CFG_TUD_*` count required). Gamepad state is sent with
  `update_xinput_state()` (`include/xinput.hpp`), and rumble/LED reports arrive via an
  `on_rumble` callback. Because a PC's XUSB driver only binds a recognized Xbox
  360 VID/PID, and because the built-in vendor class also claims interface class
  0xFF, **use X-Input as the only enabled function** (it then advertises the Xbox
  identity + 0xFF/0xFF/0xFF device class so the host recognizes it). See the
  [`xinput_example`](xinput_example/). *These are Microsoft's IDs, for emulation /
  testing of your own device only.*
- An **MSC** (mass storage) function exposing an SD card and/or a FAT partition
  in flash as USB drives, shared with the application through an ownership
  hand-over (see [Enabling mass storage](#enabling-mass-storage-msc) and the
  [`msc_example`](msc_example/)).

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
  - [Enabling the HID class](#enabling-the-hid-class)
  - [Enabling X-Input (Xbox 360)](#enabling-x-input-xbox-360)
  - [Enabling mass storage (MSC)](#enabling-mass-storage-msc)
  - [Routing the console over CDC](#routing-the-console-over-cdc)
  - [Endpoint budget (ESP32-S3 USB-OTG)](#endpoint-budget-esp32-s3-usb-otg)
  - [Example](#example)
  - [Notes](#notes)

<!-- markdown-toc end -->

## Features

- **Composable**: enable a CDC function and/or a vendor/WebUSB function and/or a
  HID function (composite).
- **Vendor-specific interface** (class 0xFF): raw bulk IN + bulk OUT byte stream.
- **HID interface**: application-supplied report descriptor (built with `hid-rp`
  in the example) on an interrupt IN endpoint; `write_hid_report()` sends reports.
- **Mass storage (MSC)**: an SD card and/or a wear-levelled FAT flash partition as
  USB drives, handed between the application (VFS file access) and the host.
- **WebUSB**: BOS + WebUSB URL + MS OS 2.0 descriptors for driverless browser
  access, with a configurable landing-page URL.
- **Console over CDC**: optionally route the ESP console (stdout) to the CDC
  interface (`CdcFunction::route_console`, or `route_console_to_cdc()`), so one
  native USB cable carries both the logs and the other interface(s) — see below.
- **Sequential allocation** of interfaces / endpoints / strings with an
  endpoint-budget check (error via `std::error_code` if exceeded).
- **Configurable identity**: VID, PID, manufacturer / product / serial / interface
  strings, plus the descriptor details some hosts check: `bcd_device`
  (device release), `max_power_ma` (bMaxPower, clamped to 500 mA and rounded up
  to the next 2 mA unit) and `remote_wakeup`.
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
// optional descriptor details (defaults: 0x0100, 100 mA, remote wakeup on);
// e.g. a Nintendo Switch expects a Pro Controller to report 0x0210 and 500 mA
cfg.bcd_device = 0x0100;
cfg.max_power_ma = 100;   // clamped to 500, rounded up to a 2 mA unit
cfg.remote_wakeup = true;

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
- `bool write_cdc(...)` / `bool write_vendor(...)` — send bytes on the respective
  interface with all-or-nothing backpressure. A frame that fits in the TX FIFO is
  written atomically: the call bounded-waits (250 ms) for room for the whole
  frame, then enqueues it in one write, so a timeout/disconnect never leaves a
  truncated prefix on the wire (returns `false` and drops the frame instead). In
  TinyUSB-callback context it fails fast if the frame does not already fit.
  Frames larger than the FIFO are streamed and are not atomic.
- `bool write_hid_report(uint8_t report_id, std::span<const uint8_t> report, ...)` —
  send a HID input report on the HID interrupt IN endpoint.
- `void set_cdc_receive_callback(...)` / `void set_vendor_receive_callback(...)`.
- `void set_mount_callback(...)` / `void set_unmount_callback(...)` — register
  device mount / unmount handlers. `esp_tinyusb` owns the raw `tud_mount_cb` /
  `tud_umount_cb`, so register here instead of defining those yourself (which
  would be a duplicate symbol). On unmount the component first clears the vendor
  + CDC TX FIFOs — so a departed host's queued backlog is not delivered to the
  next host that mounts — then invokes your callback.
- `size_t vendor_write_available() const` / `size_t cdc_write_available() const`
  and `void vendor_write_clear()` / `void cdc_write_clear()` — TX-FIFO free space
  and flush helpers (skip/defer or drop a streaming frame when the host stops
  draining).
- `bool is_cdc_connected() const` / `bool is_vendor_connected() const` /
  `bool is_hid_ready() const`.
- `bool set_msc_owner(size_t lun, MscOwner owner, ...)` — hand an MSC medium to
  the application (mounted at its `base_path`) or the host; `msc_owner(lun)`,
  `msc_capacity(lun)`, `msc_lun_count()`, `format_msc_medium(lun, ...)` and
  `set_msc_event_callback(...)` complete the MSC API.

CDC-only preset (`espp::UsbCdc`, unchanged API): `initialize()`, `write()`,
`set_receive_callback()`, `is_connected()`.

## Enabling the vendor / WebUSB class

The vendor class is gated in `esp_tinyusb` behind a Kconfig option. To use the
vendor function, set in your project's `sdkconfig.defaults`:

```
CONFIG_TINYUSB_VENDOR_COUNT=1   # THE key enablement: compiles in the vendor class
```

Setting `CONFIG_TINYUSB_VENDOR_COUNT` > 0 makes `esp_tinyusb` define
`CFG_TUD_VENDOR` and compile the TinyUSB vendor class driver. No custom
`tusb_config` is needed — the BOS descriptor and the WebUSB / MS-OS-2.0 vendor
control requests are provided by `espp::UsbDevice` through the standard TinyUSB
weak-callback overrides (`tud_descriptor_bos_cb`, `tud_vendor_control_xfer_cb`,
`tud_vendor_rx_cb`). If the vendor function is requested but `CFG_TUD_VENDOR == 0`,
`initialize()` fails with `std::errc::function_not_supported`.

CDC support is compiled conditionally (`#if CFG_TUD_CDC > 0`), so a vendor-only,
HID-only or X-Input-only build does **not** need CDC enabled. Enable it only when
you use the CDC function:

```
CONFIG_TINYUSB_CDC_ENABLED=y
CONFIG_TINYUSB_CDC_COUNT=1
```

(Requesting a CDC function while `CFG_TUD_CDC == 0` fails `initialize()` with
`std::errc::function_not_supported`, matching the vendor/HID checks.)

## Enabling the HID class

Like the vendor class, the HID class is gated in `esp_tinyusb` behind a Kconfig
option. To use the HID function, set in your project's `sdkconfig.defaults`:

```
CONFIG_TINYUSB_HID_COUNT=1   # compiles in the TinyUSB HID class driver (CFG_TUD_HID)
```

`espp::UsbDevice` provides the required TinyUSB HID weak-callback overrides
(`tud_hid_descriptor_report_cb` returns the stored report descriptor;
`tud_hid_get_report_cb` returns 0). Supply the report-descriptor bytes yourself
(the example builds them with the espp `hid-rp` component), assign them to
`HidFunction::report_descriptor`, and send input reports with
`write_hid_report(report_id, report)`.

To **receive** host→device OUTPUT / SET_REPORT reports (for request/response HID
protocols such as the Nintendo Switch Pro handshake), set `HidFunction::on_receive`
(or `set_hid_receive_callback()`) and set `HidFunction::has_out_endpoint` for
interrupt-OUT reports. The callback runs on the TinyUSB task with the report id as
byte 0 of its span; reply by sending an INPUT report with `write_hid_report()`. If
the HID function is requested but `CFG_TUD_HID == 0`, `initialize()` fails with
`std::errc::function_not_supported`.

## Enabling X-Input (Xbox 360)

X-Input needs **no** `CFG_TUD_*` count — it is served by a custom TinyUSB
application class driver built into this component (registered via the weak
`usbd_app_driver_get_cb`, forced into the link with `-u`). So an X-Input-only
project needs no CDC/vendor/HID class enabled at all; the
[`xinput_example`](xinput_example/) sdkconfig disables them:

```
CONFIG_TINYUSB_CDC_ENABLED=n
CONFIG_TINYUSB_CDC_COUNT=0
# vendor/HID counts default to 0 — importantly, keep CFG_TUD_VENDOR at 0 so the
# built-in bulk vendor driver does not claim the X-Input 0xFF interface.
```

Set `Config::xinput` (only — see the "only enabled function" note above), send
gamepad state with `update_xinput_state(GamepadState)`, and receive rumble/LED reports
via `XInputFunction::on_rumble`. The interface uses one interrupt-IN endpoint
(0x81, 20-byte input reports) and one interrupt-OUT endpoint (rumble/LED); the two
use **separate endpoint numbers**, and the DMA report buffers are word-aligned, as
the ESP32-S3 DWC2 requires. See `include/xinput.hpp` for the report/`GamepadState`
API and the button/axis layout.

## Enabling mass storage (MSC)

The MSC function exposes up to **two media** as USB drives: an SD card and/or a
FAT data partition in flash (accessed through wear levelling). It is built on
esp_tinyusb's MSC storage backend, which provides the SCSI handling, so enable it
in sdkconfig (the [`msc_example`](msc_example/) does):

```
CONFIG_TINYUSB_MSC_ENABLED=y
# flash media: the MSC buffer must hold a wear-levelling sector
CONFIG_WL_SECTOR_SIZE_512=y          # or raise CONFIG_TINYUSB_MSC_BUFSIZE to 4096
```

**Ownership.** A medium belongs to one side at a time, so the firmware and a PC
never write the same FAT volume at once:

- While the **application** owns it, the FAT volume is mounted at the medium's
  `base_path` and you use ordinary file APIs (`fopen`, `std::fstream`,
  `std::filesystem`). A connected host sees the drive as "no medium".
- While the **host** owns it, `base_path` is unmounted (files you had open there
  become invalid) and the PC sees the volume.

With `MscFunction::auto_handover` (the default) the host takes the media when it
mounts the device, and the application gets them back when the host ejects the
drive or the device is detached. To finish application I/O before any host can
take a medium, set `Config::connect_on_initialize = false` and call `connect()`
when done (the `msc_example` does). The hand-over covers **all media at once**:
esp_tinyusb ignores which drive was ejected, so ejecting either returns both.
Taking a medium from an attached host with `set_msc_owner()` is refused
(`device_or_resource_busy`) because host writes already queued could land under
the application's volume: eject the drive on the host first. Turn it off to decide yourself with
`set_msc_owner(lun, MscOwner::Host / App)` — for example only expose an SD card
while a "USB drive" screen is shown. `msc_owner()`, `msc_capacity()` and
`MscFunction::on_event` (hand-over started / done / failed, format required)
report the state; the event callback runs in the TinyUSB task for host-driven
hand-overs, so act on it from your own task.

```cpp
espp::UsbDevice::MscMedium card;
card.type = espp::UsbDevice::MscMedium::Type::SdCard;
card.sd_card = sd_card;       // an initialized sdmmc_card_t* (SDMMC or SDSPI host)
card.base_path = "/sdcard";   // do NOT also esp_vfs_fat_*_mount() the card yourself

espp::UsbDevice::MscMedium flash;
flash.type = espp::UsbDevice::MscMedium::Type::FlashPartition;
flash.partition_label = "storage"; // a `data, fat` partition
flash.base_path = "/data";
flash.volume_label = "MY DATA";   // drive name on the host; needs CONFIG_FATFS_USE_LABEL=y

espp::UsbDevice::MscFunction msc;
msc.media = {card, flash};         // LUN 0 and LUN 1
cfg.msc = msc;
```

Limits, all from esp_tinyusb's backend: at most one SD card and one flash
partition; SD card media need a target with an SDMMC host peripheral (ESP32-S3 /
-P4), even for an SPI-wired card; the SCSI inquiry strings are esp_tinyusb's
fixed ones. **Formatting** (`format_if_unformatted` / `format_msc_medium()`) runs
on FatFs drive 0 rather than the medium's own drive, so only use it when no other
FAT volume is mounted on the device. A host can only read FAT, so a LittleFS (or
SPIFFS) partition cannot be exposed as a drive — use a FAT partition for storage
you want to share with a PC.

Destroying the `UsbDevice` releases the media: an application-owned medium's
`base_path` is unmounted (an SD card stays initialized, but you must mount it
again if the application still needs its files).

## Routing the console over CDC

When the native USB port is handed to TinyUSB for a vendor / HID / XInput
interface, the ESP console can no longer live on **USB-Serial-JTAG** — on the
ESP32-S3 that controller shares the same USB PHY as USB-OTG, so a console on it
contends with the TinyUSB interface and reboot-loops the device. Add a **CDC**
function and route the console to it, and a single native USB cable carries both
the logs and the other interface(s):

```cpp
espp::UsbDevice::CdcFunction cdc;
cdc.route_console = true;   // redirect stdout -> this CDC interface after initialize()
// cdc.tee_console = true;  // (default) also keep the primary UART console (idf.py monitor)
usb_cfg.cdc = cdc;
usb_cfg.vendor = my_vendor; // or hid / xinput -- CDC is just the log channel
espp::UsbDevice usb(usb_cfg);
usb.initialize(ec);         // console is now on CDC (teed to UART)
```

Or call it yourself for control over timing: `usb.route_console_to_cdc()` after a
successful `initialize()`.

- `printf`, `ESP_LOG` (its default vprintf), and `espp::Logger` (which uses
  `fmt::print`) all write to `stdout`, so redirecting **stdout** captures every
  console path. A tiny write-only VFS device is registered and `stdout` is
  `freopen`ed onto it.
- Writes are **non-blocking**: a chunk is mirrored to CDC only if it fits the TX
  FIFO right now (so an absent / slow reader never stalls a logging task); it is
  not gated on DTR, so a plain serial monitor still sees output.
- With `tee_console` (default) the console is also written to the primary **UART**
  console when there is one, so `idf.py monitor` on UART keeps working and nothing
  is lost when no CDC host is attached. (There is nothing to tee to for a
  USB-Serial-JTAG or `CONSOLE_NONE` console.)
- Recommended sdkconfig: primary console on **UART0**
  (`CONFIG_ESP_CONSOLE_UART_DEFAULT`), optionally USB-Serial-JTAG as the
  **secondary** console for early-boot logs before TinyUSB comes up.

The `ota` example uses this to carry its logs alongside the OTA vendor / WebUSB
interface on one cable.

## Endpoint budget (ESP32-S3 USB-OTG)

The ESP32-S3 / -S2 USB-OTG core is full-speed and, besides EP0, provides roughly
**5 usable data IN endpoints** and **5 usable data OUT endpoints**. Each function
consumes:

| Function          | IN endpoints                                | OUT endpoints                  |
|-------------------|---------------------------------------------|--------------------------------|
| CDC-ACM           | 2 (1 interrupt-IN notif + 1 bulk-IN)        | 1 (bulk-OUT)                   |
| Vendor / WebUSB   | 1 (bulk-IN)                                  | 1 (bulk-OUT)                   |
| HID               | 1 (interrupt-IN)                            | 0 or 1 (optional interrupt-OUT) |
| X-Input (Xbox 360)| 1 (interrupt-IN)                            | 1 (interrupt-OUT)              |
| MSC               | 1 (bulk-IN)                                  | 1 (bulk-OUT)                   |

This is why the device is **selectable** ("not all at once"). Combinations that
fit comfortably: CDC+Vendor (3 IN / 2 OUT, used by the example), CDC+Vendor+HID,
CDC+Vendor+MSC. Enabling CDC+Vendor+HID+MSC reaches 5 IN endpoints — at the hard
limit, not recommended. `initialize()` returns `std::errc::value_too_large` if the
IN or OUT budget is exceeded.

## Example

See [`msc_example/`](msc_example/) for a USB drive backed by a flash FAT partition,
[`xinput_example/`](xinput_example/) for an Xbox 360 controller, and `example/` for a full project that wires a **composite CDC + Vendor/WebUSB**
`espp::UsbDevice` to the transport-agnostic `espp::OdriveAscii` protocol server.
Both interfaces feed the same server (RX from either interface → `process_bytes`
→ response written back out the same interface), while the log console stays on
the USB-Serial-JTAG peripheral.

## Notes

- USB-OTG is only available on the ESP32-S2, ESP32-S3 and ESP32-P4 targets.
- Only one `espp::UsbDevice` / `espp::UsbCdc` instance may exist at a time.
- The receive callbacks run in the TinyUSB device task; keep them short and
  non-blocking.
- The TinyUSB device lifecycle callbacks (`tud_mount_cb` / `tud_umount_cb` /
  `tud_suspend_cb` / `tud_resume_cb`) are owned by `esp_tinyusb`. Register mount
  / unmount handlers via `set_mount_callback()` / `set_unmount_callback()`
  rather than defining those callbacks yourself. The mount / unmount handlers
  also run in the TinyUSB device task.
