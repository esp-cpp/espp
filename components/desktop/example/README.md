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

Hardware apps, each behind a Kconfig option (all on by default, so the CI
build compiles every one of them; see [Configuration](#configuration)):

- **CANopen / DS402** — a CiA 301 NMT master + SDO client
  (`espp::CanopenClient`) and a CiA 402 drive panel (`espp::Ds402Drive`):
  node id, NMT Start / Stop / Pre-operational / Reset, NMT + drive state and
  statusword, mode of operation, Enable / Disable / Quick stop / Fault reset,
  a target-velocity slider, position / velocity, and a raw SDO read / write
  row. The bus is either the in-firmware **simulated DS402 node** (the CAN
  bridge example's `SimulatedCanBus`, no hardware needed, with an "Inject
  fault" button) or the **TWAI peripheral** wired to a CAN transceiver. Every
  bus transaction runs on the app's own task (SDO calls block); the window
  only queues commands and the task updates the widgets.
- **I2C scanner** — probe every 7-bit address on the configured bus
  (`espp::I2c`) from a short task and list what answers; read / write a
  device register from the window. A bus that fails to initialize shows a
  hint instead.
- **Network** — the Wi-Fi station (`espp::WifiSta`): status / SSID / IP /
  RSSI / MAC, Scan (on its own task; a scan disconnects first), the AP list,
  password field and Connect / Disconnect / Forget with the credentials kept
  in NVS (`desktop` namespace, `wifi_ssid` / `wifi_pass`); and, on SoCs with an
  EMAC, the RMII Ethernet link (`espp::Ethernet`): link / IP / MAC / speed.
  The interfaces come up on the first launch and stay up when the window is
  closed.

## How to use example

### Hardware Required

An ESP32-S3 (or -S2 / -P4) board with the native USB port wired to a host. The
console / logs go to UART0 (see `sdkconfig.defaults`); with
`CONFIG_DESKTOP_EXAMPLE_LOG_CAPTURE` (default on) they are also captured for
the Log Viewer.

The hardware apps need nothing extra by default: the CANopen app talks to a
simulated node, the I2C scanner just reports an empty bus and the Network app
scans for Wi-Fi. For a real CAN bus select the TWAI peripheral and wire a
transceiver (SN65HVD230 or similar) to the configured TX / RX GPIOs; for the
Ethernet group an ESP32-Ethernet-Kit-style RMII PHY (the pins are in
`main/apps/network_app.hpp`).

### Configuration

`idf.py menuconfig` -> *Desktop Example Configuration*:

| Option | Default | Meaning |
|---|---|---|
| `DESKTOP_EXAMPLE_LOG_CAPTURE` (+ `_BYTES`) | y (16384) | Tee the console into a ring for the Log Viewer |
| `DESKTOP_EXAMPLE_ENABLE_CANOPEN` | y | Register the CANopen / DS402 app |
| `DESKTOP_EXAMPLE_CANOPEN_BUS` | `SIMULATED` | `SIMULATED` (in-firmware DS402 node) or `TWAI` (the peripheral) |
| `DESKTOP_EXAMPLE_CANOPEN_NODE_ID` | 1 | Server node id (1..127; also the simulated node's id) |
| `DESKTOP_EXAMPLE_CAN_TX_GPIO` / `_RX_GPIO` / `_BAUDRATE` | 17 / 16 / 500000 | TWAI wiring and bit rate (TWAI bus only) |
| `DESKTOP_EXAMPLE_ENABLE_I2C` | y | Register the I2C scanner app |
| `DESKTOP_EXAMPLE_I2C_PORT` / `_SDA_GPIO` / `_SCL_GPIO` / `_FREQ_HZ` | 0 / 8 / 9 / 400000 | The I2C bus it scans |
| `DESKTOP_EXAMPLE_ENABLE_WIFI` | y | The Network app's Wi-Fi station group (`SOC_WIFI_SUPPORTED`) |
| `DESKTOP_EXAMPLE_ENABLE_ETHERNET` | n | The Network app's RMII Ethernet group (`SOC_EMAC_SUPPORTED`: ESP32 / -P4) |

The hardware components (`canopen`, `twai`, `i2c`, `wifi`, `ethernet`, `cli`)
are always part of the build (`REQUIRES` cannot depend on Kconfig); the
options only decide which apps are registered. The simulated CAN bus /
DS402 node headers are included from the CAN bridge example
(`components/canopen/can_bridge_example/main`); promoting them into the
`canopen` component is a follow-up.

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
