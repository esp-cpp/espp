# VL53L5CX Web Console Example

Turns an ESP32-S3 / ESP32-P4 into a **WebUSB / Web Serial front-end** for a
VL53L5CX (or VL53L7CX) multizone Time-of-Flight sensor: the hosted
[VL53L5CX console web app](https://esp-cpp.github.io/espp/apps/vl53l5cx_console.html)
connects over native USB and can

- **view** every ranging frame live, as a 3D surface (each zone at its
  measured position in the sensor's field of view) or a bar height field,
  coloured by distance / signal / ambient / sigma / reflectance / status, next
  to a numeric grid with per-zone details and the motion indicator,
- **configure** the sensor — resolution (4x4 / 8x8), ranging frequency,
  integration time, sharpener, target order, ranging mode, power mode, the
  motion indicator window, the I2C address — and
- **control** it: start / stop ranging, re-initialize (firmware upload).

The example runs the `espp::Vl53l5cx` driver **on the device** and exposes a
small high-level protocol (`vl53l5cx_protocol.hpp`, dispatcher **module id
10**, protocol id `espp.vl53l5cx`) through `espp::Vl53l5cxService`, so the web
app needs no knowledge of the sensor's I2C / firmware interface. Both the
vendor (WebUSB) and CDC (Web Serial) interfaces carry the same protocol; the
system console/logs go to **UART0** (set in `sdkconfig.defaults`): on the S3 /
P4, USB-Serial-JTAG shares the native USB port's PHY with USB-OTG, which
TinyUSB takes over.

## Wiring & configuration

The sensor sits on I2C (3.3 V, GND, SDA, SCL; LPn high or left to the
breakout's pull-up). `idf.py menuconfig` → *Example Configuration* selects the
hardware (QtPy ESP32-S3: SDA 41 / SCL 40; ESP32-P4 dev kits: SDA 7 / SCL 8; or
custom pins), the I2C clock, the **part** (VL53L5CX / VL53L7CX — same ids,
different firmware image), the resolution and frequency at boot, whether
ranging starts at boot, and on the ESP32-P4 which **USB port** TinyUSB drives
(the high-speed port by default; the full-speed port is shared with the
USB-Serial-JTAG console).

## Protocol (module 10, `espp.vl53l5cx` v1)

Framed with `stream_frame` and routed by `espp::Dispatcher`. Requests use
types `0x1X`; replies / events use `0x9X` (reply flag set). Multi-byte fields
are little-endian.

| Type | Dir | Meaning |
|------|-----|---------|
| `0x10` GET_STATUS | H→D | request one STATUS snapshot |
| `0x11` SET_RESOLUTION | H→D | `[zones u8]` 16 (4x4) or 64 (8x8) — not while ranging |
| `0x12` SET_RANGING_FREQUENCY | H→D | `[hz u8]` 1..60 (4x4) / 1..15 (8x8) |
| `0x13` SET_INTEGRATION_TIME | H→D | `[ms u32]` 2..1000 (autonomous mode) |
| `0x14` SET_SHARPENER | H→D | `[percent u8]` 0..99 |
| `0x15` SET_TARGET_ORDER | H→D | `[order u8]` 1 closest / 2 strongest |
| `0x16` SET_RANGING_MODE | H→D | `[mode u8]` 1 continuous / 3 autonomous |
| `0x17` SET_POWER_MODE | H→D | `[mode u8]` 0 sleep / 1 wake-up |
| `0x18` ENABLE_MOTION_INDICATOR | H→D | `[min_mm u16][max_mm u16]` (only INITIALIZE switches it off) |
| `0x19` START_RANGING | H→D | start ranging with the current settings |
| `0x1A` STOP_RANGING | H→D | stop ranging |
| `0x1B` INITIALIZE | H→D | reboot the sensor + re-upload the firmware (~2 s); resets every setting |
| `0x1C` SET_FRAME_STREAM | H→D | `[enable u8][divider u8]` stream every divider-th frame on this transport |
| `0x1D` GET_FRAME | H→D | request the latest FRAME |
| `0x1E` SET_I2C_ADDRESS | H→D | `[address u8]` 7-bit, volatile |
| `0x90` STATUS | D→H | `[flags u8][variant u8][address u8][zones u8][frequency u8][integration_ms u32][sharpener u8][target_order u8][ranging_mode u8][power_mode u8][motion_min u16][motion_max u16][frame_count u32][frame_rate_cHz u16][stream_divider u8]` — flags: bit 0 initialized, 1 ranging, 2 motion indicator on, 3 stream on |
| `0x91` FRAME | D→H | `[seq u32][time_ms u32][zones u8][flags u8]`, then per zone `[distance i16][status u8][targets u8][reflectance u8][sigma u16][signal u32][ambient u32][spads u32]` (19 B, row-major), then the 140-byte motion block when flags bit 0 is set |
| `0x92` OK | D→H | `[request_type u8]` |
| `0x93` ERROR | D→H | `[request_type u8][code u32][utf8 message]` |

Settings are refused by the sensor while it is ranging (the device answers
ERROR); STATUS reports the settings read back from the sensor when it is idle
and the last known values while it ranges.

## Build & flash

```
idf.py set-target esp32s3        # or esp32p4
idf.py build flash monitor       # console is on UART0
```

Then open the VL53L5CX console web app and Connect (WebUSB or Web Serial).
The sensor ranges at boot by default: tick **Live frames** (on by default) to
see the 3D view; **Stop** to change the settings, then **Start ranging**.

## Standard USB services

Like every espp USB example, this one serves the standard service set on its
framed USB link(s) next to its own protocol, so the hosted consoles and the
[Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html) (which
finds each service through discovery, by protocol id) work against it:

| Service | Module (default) | Protocol id | Console |
|---|---|---|---|
| `espp::SystemService` -- device info, reboot, reboot into the bootloader | 7 | `espp.system` | [system console](https://esp-cpp.github.io/espp/apps/system_console.html) |
| `espp::MonitorService` -- heap regions + task table, on request or streamed | 8 | `espp.monitor` | system console |
| `espp::OtaService` -- firmware update (host-driven rollback confirmation) | 0 | `espp.ota` | [OTA console](https://esp-cpp.github.io/espp/apps/ota_console.html) |
| `espp::CoreDumpService` -- last-crash report, core dump download / erase | 4 | `espp.coredump` | [coredump console](https://esp-cpp.github.io/espp/apps/coredump_console.html) |

`partitions.csv` therefore carries the OTA layout (`otadata`, `ota_0`, `ota_1`)
plus a `coredump` partition, and `sdkconfig.defaults` enables core dumps to
flash, OTA rollback and the FreeRTOS run-time statistics the task monitor
reads. Every device->host write on a transport goes through one mutex, so the
services (and the frame stream) never interleave frames.
