# USB Device Example (CDC + Vendor/WebUSB + MSC, with the standard espp USB services)

The reference `espp::UsbDevice` example: **one composite USB device with the
three interface classes the component provides**, and the standard espp USB
services on every framed link, so the hosted web consoles, the Device Hub and
the `espp_ota` / `espp_coredump` command-line tools all work against it.

| Interface | What it carries | Talk to it with |
|-----------|-----------------|-----------------|
| **CDC-ACM** (a serial port) | the espp framed protocol (`stream_frame`, routed by an `espp::DispatcherWorker`) | the hosted consoles over **Web Serial**; the `espp_ota` / `espp_coredump` CLIs over the serial port |
| **vendor-specific (class 0xFF, WebUSB)** | the same framed protocol over bulk IN/OUT | the hosted consoles over **WebUSB** (the BOS landing page points at the system console); the CLIs over libusb |
| **MSC** (a USB drive) | a wear-levelled FAT partition in flash (`storage`, 896K), with a `README.txt` the firmware writes at boot | mount it like any removable drive; eject it to hand it back to the firmware |

The device enumerates as VID `0x1209` / PID **`0x0d38`** (manufacturer "espp",
product "espp USB Device"); the PID is distinct from the other espp examples so a
host-side filter can be specific. The log console is on **UART0** (USB-Serial-JTAG
as the early-boot secondary): on the ESP32-S3 the USB-Serial-JTAG controller and
USB-OTG share the same native USB PHY, so the console cannot stay there once
TinyUSB owns the port.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [USB Device Example (CDC + Vendor/WebUSB + MSC, with the standard espp USB services)](#usb-device-example-cdc--vendorwebusb--msc-with-the-standard-espp-usb-services)
  - [Standard USB services](#standard-usb-services)
  - [The USB drive](#the-usb-drive)
  - [Partition layout](#partition-layout)
  - [Build, flash, run](#build-flash-run)
  - [How it works](#how-it-works)

<!-- markdown-toc end -->

## Standard USB services

Both framed links (CDC and vendor) serve the same set, each service registered
on each link's dispatcher and advertised through capability discovery, so the
[Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html) lists them
and every console finds its module by protocol id (the module ids below are the
published defaults; hosts do not depend on them):

| Service | Protocol id | Module | Console / tool |
|---------|-------------|--------|----------------|
| `espp::SystemService` — chip / firmware / partition info, reboot, reboot into the ROM bootloader (download mode) | `espp.system` v1 | 7 | [system console](https://esp-cpp.github.io/espp/apps/system_console.html) |
| `espp::MonitorService` — heap regions and the task table, on request or streamed | `espp.monitor` v1 | 8 | system console |
| `espp::OtaService` — firmware update into the other OTA slot, with rollback confirmation | `espp.ota` v1 | 0 | [OTA console](https://esp-cpp.github.io/espp/apps/ota_console.html), `espp_ota` / `idf.py ota-usb` |
| `espp::CoreDumpService` — last-crash report, core dump download / erase | `espp.coredump` v1 | 4 | [coredump console](https://esp-cpp.github.io/espp/apps/coredump_console.html), `espp_coredump` / `idf.py coredump-usb` |

Reboot requests go through the example's `on_reboot_request` callback, which
logs and permits them; an application would refuse or defer one while, say, the
host is writing to the drive.

## The USB drive

The MSC function exposes the `storage` partition (a `data, fat` partition in
`partitions.csv`, accessed through wear levelling) as a removable drive named
`ESPP USB`. Ownership follows the `usb_device` MSC model: the firmware owns the
volume first (it formats it if needed and writes `README.txt`), the host takes
it when it mounts the device, and ejecting the drive on the host gives it back
to the firmware (logged by the heartbeat). The firmware and the host never
write the volume at the same time.

## Partition layout

`partitions.csv` (4 MB flash): `nvs`, `otadata`, `phy_init`, two 1536K app slots
`ota_0` / `ota_1` (`idf.py flash` writes `ota_0`, each OTA update alternates to
the other slot), a 64K `coredump` partition and the 896K `storage` FAT volume.

## Build, flash, run

```sh
cd components/usb_device/example
idf.py set-target esp32s3
idf.py build flash monitor   # console is on UART0 (USB-UART adapter)
```

Then connect the native USB port: the host sees a serial port, a WebUSB
interface and a drive. Open the
[system console](https://esp-cpp.github.io/espp/apps/system_console.html) (WebUSB
or Web Serial) for the device info, the reboot buttons and the live heap / task
view, the OTA and coredump consoles for updates and crash dumps, and mount the
drive to read the README the firmware wrote.

## How it works

- `espp::UsbDevice` installs the TinyUSB driver and builds the descriptors for
  the enabled CDC + vendor + MSC functions, allocating interfaces / endpoints
  sequentially; the vendor function advertises WebUSB + MS OS 2.0 descriptors
  so a browser (and Windows, via WinUSB) can bind it driverlessly.
- Each framed link has its own `espp::DispatcherWorker` (a bounded receive queue
  + worker task feeding one `espp::Dispatcher`), fed from the TinyUSB receive
  callbacks; the four services are registered on each worker and
  `serve_discovery()` answers the hub's query. Every device->host write on a
  transport goes through one application-level mutex, so a streamed monitor
  event and a reply from another service never interleave.
- The OTA and core-dump engines (`espp::Ota`, `espp::CoreDump`) are shared by
  the per-link services; an RX overflow on a link aborts an OTA transfer that
  link owned and tells the host.
- The MSC medium is configured with the application as the initial owner and
  `connect_on_initialize = false`, so the README is written before the device
  presents itself to the host; `auto_handover` then moves the drive to the host
  on mount and back on eject.
