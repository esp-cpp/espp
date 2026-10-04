# Desktop over USB Example

A browser-rendered windowed desktop served from an ESP32-S3: the firmware
registers apps and describes their windows / widgets with `espp::Desktop`;
the hosted [desktop web app](https://esp-cpp.github.io/espp/apps/desktop.html)
(`components/desktop/web/desktop.html`) draws and operates them over the
native USB port, on both the **vendor (WebUSB)** and **CDC (Web Serial)**
interfaces (`espp.desktop` v1 on module 9 by default). The
[Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html) lists it
through discovery next to the standard services.

Apps (`main/apps/*.hpp`, one `register_<name>_app()` each):

- **Counter** — the API reference (~40 lines): a label, three buttons, the
  count kept in NVS, a confirmation message box.
- **About** — chip / firmware / partition / hardware labels (`SystemInfo`).
- **System Monitor** — uptime and per-region heap gauges (`HeapMonitor`),
  refreshed by a 1 s window timer.
- **Task Manager** — the FreeRTOS task table (`TaskMonitor`: CPU %, stack
  high-water mark, priority, core) with a refresh-period selector. Filter by task name (substring,
  case-insensitive) and core; click a column header to sort (again for
  descending, again to clear).
- **Log Viewer** — the captured console (`ConsoleCapture`) streamed live into
  a read-only, ANSI-aware console text area; pause and clear.
- **Files** — browse the LittleFS partition (`FileSystem`), create / rename /
  delete through dialogs; open a file in the **Editor** (a text area saved
  with `std::ofstream`).
- **Settings** — nickname, theme and accent (applied to the browser at once)
  and the log-capture tee (whether captured logs still go to the UART
  console; the capture itself is the compile-time
  `CONFIG_DESKTOP_EXAMPLE_LOG_CAPTURE`), all kept in NVS and restored at boot.

## How to use example

### Hardware Required

An ESP32-S3 (or -S2 / -P4) board with the native USB port wired to a host. The
console / logs go to UART0 (see `sdkconfig.defaults`); with
`CONFIG_DESKTOP_EXAMPLE_LOG_CAPTURE` (default on) they are also captured for
the Log Viewer.

### Build and Flash

```
idf.py set-target esp32s3
idf.py build flash monitor
```

CI builds it with the component manager off (`IDF_COMPONENT_MANAGER=0 idf.py
build`), resolving every dependency from the repository (including the
vendored `esp_tinyusb` / `tinyusb` submodules under `external/` and the
`littlefs` submodule under `components/`).

Then open the desktop web app and Connect (WebUSB or Web Serial): the app
icons appear; double-click one (or use the start menu) to launch it. Windows
can be dragged, resized, minimised, maximised and closed; the browser
remembers where you put them. Reconnecting (or reloading the page) resyncs
the whole desktop with one `GET_DESKTOP`.

## Standard USB services

Like every espp USB example, this one serves the standard service set on its
framed USB link(s) next to its own protocol, so the hosted consoles and the
[Device Hub](https://esp-cpp.github.io/espp/apps/dispatcher_hub.html) (which
finds each service through discovery, by protocol id) work against it:

| Service | Module (default) | Protocol id | Console |
|---|---|---|---|
| `espp::DesktopService` -- this desktop | 9 | `espp.desktop` | [desktop](https://esp-cpp.github.io/espp/apps/desktop.html) |
| `espp::SystemService` -- device info, reboot, reboot into the bootloader | 7 | `espp.system` | [system console](https://esp-cpp.github.io/espp/apps/system_console.html) |
| `espp::MonitorService` -- heap regions + task table, on request or streamed | 8 | `espp.monitor` | system console |
| `espp::OtaService` -- firmware update (host-driven rollback confirmation) | 0 | `espp.ota` | [OTA console](https://esp-cpp.github.io/espp/apps/ota_console.html) |
| `espp::CoreDumpService` -- last-crash report, core dump download / erase | 4 | `espp.coredump` | [coredump console](https://esp-cpp.github.io/espp/apps/coredump_console.html) |

`partitions.csv` therefore carries the OTA layout (`otadata`, `ota_0`, `ota_1`),
a `coredump` partition and a `littlefs` partition for the Files app, and
`sdkconfig.defaults` enables core dumps to flash, OTA rollback and the FreeRTOS
run-time statistics the task monitor reads. Every device->host write on a
transport goes through one mutex, so the services never interleave frames;
`write_vendor` / `write_cdc` wait (bounded, 250 ms) for FIFO room for a whole
frame and never queue a partial one, and when the host is not draining the
FIFO the frame is dropped and the desktop flags that transport as needing a
resync (the browser resyncs with GET_DESKTOP on its next connect).

## Example Output

```
I (327) Desktop Example: Starting desktop example
I (337) Desktop Example: LittleFS at /littlefs: 8 / 256 KiB used
I (347) Desktop Example: Clean boot history (reset reason: power-on)
I (1077) Desktop Example: Ready. Connect the native USB port and open the desktop ...
```
