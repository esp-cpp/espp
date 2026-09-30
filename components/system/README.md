# System Component

[![Badge](https://components.espressif.com/components/espp/system/badge.svg)](https://components.espressif.com/components/espp/system)

The `system` component answers "what is this device, how is it doing, and can
I restart it?" for any espp application:

- `espp::SystemInfo` — static getters (and a one-call `collect()` /
  `to_string()` snapshot) for the chip model / revision / cores / features, the
  ESP-IDF version, the application description (project name, version, build
  date and time, ELF SHA-256), the running and boot partitions with the OTA
  image state, the reset reason, uptime, base MAC, flash and PSRAM sizes, CPU
  frequency and the free / minimum-free heap.
- `espp::SystemControl` — `reboot()` and `reboot_to_bootloader()`: the latter
  sets the chip's *force download boot* flag and restarts, so the device comes
  back in the ROM bootloader's download mode ready for `esptool` / `idf.py
  flash` (what holding the BOOT strap does, without a button). Supported on the
  ESP32-S2 / -S3 (the ROM's USB CDC / DFU device stays attached), -C2 / -C3 /
  -C5 / -C6 / -C61 / -H2 / -H21 and -P4 (USB-Serial-JTAG); the classic ESP32
  has no software path and reports `operation_not_supported`.
- `espp::SystemService` — both of the above as a transport-agnostic
  [dispatcher](../dispatcher) module (`espp.system` v1, module 7 by default):
  `GET_INFO` answers with a tagged-record snapshot hosts can extend-proof
  decode, `REBOOT` / `REBOOT_TO_BOOTLOADER` reply OK and restart after a delay.
  Both restarts are guarded by `Config::allow_reboot` / `allow_bootloader` and
  an optional `on_reboot_request` veto callback, so an application can refuse a
  restart while, say, a motor is running.

The hosted [system console](https://esp-cpp.github.io/espp/apps/system_console.html)
web app (`web/system_console.html`) talks to the service over WebUSB or Web
Serial, and to the [monitor](../monitor) component's `MonitorService` (heap
regions + task table, live) when the device advertises it.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [System Component](#system-component)
  - [Protocol (module 7, `espp.system` v1)](#protocol-module-7-esppsystem-v1)
  - [Example](#example)

<!-- markdown-toc end -->

## Protocol (module 7, `espp.system` v1)

Framed with `stream_frame` and routed by `espp::Dispatcher`; requests carry the
reply flag clear, replies set it (type high bit). All multi-byte fields are
little-endian. See `include/detail/system_protocol.hpp` (host-buildable, with
`test/system_host_test.cpp`).

| Type | Dir | Meaning |
|------|-----|---------|
| `0x01` GET_INFO | H→D | request an INFO reply |
| `0x02` REBOOT | H→D | `[delay_ms u16]` — reply OK, restart after the delay |
| `0x03` REBOOT_TO_BOOTLOADER | H→D | `[delay_ms u16]` — reply OK, restart into download mode |
| `0x81` INFO | D→H | tagged records `[tag u8][len u8][value]` (unknown tags are skipped) |
| `0x83` OK | D→H | `[request_type u8]` |
| `0x84` ERROR | D→H | `[request_type u8][code u32][utf8 message]` — code is the POSIX errno of the chosen `std::errc` (the message is authoritative) |

INFO tags: 1 chip model (str), 2 chip revision (u16), 3 cores (u8), 4 chip
features (u32), 5 IDF version, 6 project name, 7 app version, 8 build date, 9
build time, 10 ELF SHA-256 (32 bytes), 11 running partition, 12 boot partition,
13 OTA state (u8), 14 reset reason (u8), 15 uptime ms (u64), 16 MAC (6 bytes),
17 flash size, 18 PSRAM size, 19 CPU MHz, 20 free heap, 21 min free heap (all
u32), 22 capabilities (u32: bit0 reboot allowed, bit1 bootloader reboot allowed
and supported). The delay is clamped to at least `Config::min_restart_delay`
so the OK reply leaves the transport before the restart.

## Example

The [example](./example) exposes `SystemService` and `MonitorService` on the
native USB port (vendor / WebUSB + CDC / Web Serial) of an ESP32-S3 for the
system console web app.
