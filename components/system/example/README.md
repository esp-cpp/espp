# System Info + Control over USB Example

Exposes the espp `SystemService` (device identity / status, reboot, reboot
into the ROM bootloader) and `MonitorService` (heap regions and the task
table, on request or streamed) on the native USB port of an ESP32-S3, over
both the **vendor (WebUSB)** and **CDC (Web Serial)** interfaces. The hosted
[system console web app](https://esp-cpp.github.io/espp/apps/system_console.html)
(`components/system/web/system_console.html`) talks to both services; the
[Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html) lists them
through discovery (`espp.system` v1 on module 7, `espp.monitor` v1 on module 8
by default).

## How to use example

### Hardware Required

An ESP32-S3 (or -S2 / -P4) board with the native USB port wired to a host. The
system console / logs go to UART0 (see `sdkconfig.defaults`).

### Build and Flash

```
idf.py set-target esp32s3
idf.py build flash monitor
```

CI builds it with the component manager off (`IDF_COMPONENT_MANAGER=0 idf.py
build`), resolving every dependency from the repository (including the
vendored `esp_tinyusb` / `tinyusb` submodules under `external/`).

Then open the system console web app and Connect (WebUSB or Web Serial):

- **Device info**: chip, ESP-IDF version, application (project, version, build
  date / time, ELF SHA-256), running / boot partition and OTA state, reset
  reason, uptime, MAC, flash / PSRAM size, CPU frequency, heap.
- **Reboot** and **Reboot into bootloader**: the device replies OK and restarts
  after the delay; in the second case it comes back in the ROM download mode
  (the S3's ROM USB CDC / DFU interface) ready for `esptool` / `idf.py flash`.
  The example's `on_reboot_request` callback logs and permits every request.
- **Heap** gauges per region and a **live task table** (CPU %, stack
  high-water mark, priority, core) with a stream toggle and period.

Task statistics need `CONFIG_FREERTOS_USE_TRACE_FACILITY` and
`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` (set in `sdkconfig.defaults`).

## Example Output

```
I (317) System Example: Starting system info + control example
I (327) System Example: System:
ESP32-S3 rev 0.2 (2 cores), ESP-IDF v6.1
app: system_example 1 built Sep 30 2026 12:34:56
partition: running 'factory', boot 'factory', OTA state n/a
reset: power-on; uptime 320 ms; MAC 34:85:18:xx:xx:xx
flash 8192 KiB, PSRAM 0 KiB, CPU 240 MHz, heap free 318412 (min 318412)
I (357) System Example: Reboot into the bootloader is supported on this chip
I (1077) System Example: Ready. Connect the native USB port and open the system console ...
```
