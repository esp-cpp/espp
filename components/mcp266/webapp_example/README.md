# MCP266 Web Console Example

Turns an ESP32-S3 into a **WebUSB / Web Serial front-end** for a Basicmicro
MCP266 motor controller: the hosted
[MCP266 console web app](https://esp-cpp.github.io/espp/apps/mcp266_console.html)
connects over native USB and can

- **configure** each axis' position loop (clamp + fallback P gain) and CiA 402
  software limits, and clear faults / e-stop,
- **command** profile-position moves (target, velocity, accel, decel), and
- **view** live per-axis status — position, velocity, DS402 state, target-reached
  — plus device telemetry (battery voltage, temperature).

Unlike the [CAN bridge](../../canopen/can_bridge_example) (which forwards raw CAN
and runs CANopen in the browser), this example runs the `espp::Mcp266` driver
**on the device** and exposes a small high-level protocol (see
`main/mcp266_protocol.hpp`, dispatcher **module id 6**), so the web app needs no
CANopen/DS402 knowledge. Both the vendor (WebUSB) and CDC (Web Serial) interfaces
carry the same protocol; the system console/logs stay on the built-in
USB-Serial-JTAG.

## Wiring & configuration

The ESP32-S3 is the CANopen **master** of the MCP266 node. Connect the TWAI
TX/RX GPIOs to a 3.3 V CAN transceiver (e.g. SN65HVD230) on a 120 Ω-terminated
bus, at the baudrate configured on the MCP266 in Basicmicro Motion Studio.

Defaults (change in `main/mcp266_webapp_example.cpp`):

| Setting | Value |
|---------|-------|
| TWAI TX | GPIO 17 |
| TWAI RX | GPIO 16 |
| CAN baudrate | 1000000 |
| MCP266 node id | 10 |

## Protocol (module 6)

Framed with `stream_frame` and routed by `espp::Dispatcher`. Requests use type
high-nibble 6; replies/events use high-nibble E (reply flag set).

| Type | Dir | Meaning |
|------|-----|---------|
| `0x60` START | H→D | NMT-start the node + clear faults |
| `0x61` RESET_FAULTS | H→D | clear latched CiA 402 faults (both axes) |
| `0x62` RESET_ESTOP | H→D | attempt an e-stop reset |
| `0x63` CONFIGURE_POSITION_LOOP | H→D | `[axis u8][min i32][max i32][fallback_p i32]` |
| `0x64` SET_POSITION_LIMITS | H→D | CiA 402 software limits: `[axis u8][min i32][max i32]` |
| `0x65` MOVE_TO_POSITION | H→D | `[axis u8][target i32][vel u32][accel u32][decel u32]` |
| `0x66` DRIVE_SPEED | H→D | `[axis u8][qpps i32]` (inert on tested firmware) |
| `0x67` DRIVE_DUTY | H→D | `[axis u8][duty i16]` (inert on tested firmware) |
| `0x68` GET_STATUS | H→D | request one STATUS snapshot |
| `0x69` SET_STATUS_STREAM | H→D | `[enable u8][period_ms u16]` |
| `0x6A` GET_DEVICE_INFO | H→D | request DEVICE_INFO |
| `0xE0` STATUS | D→H | per-axis `[pos i32][vel i32][statusword u16]` ×2, then `[battery_dV u16][temp_dC u16][flags u8]` |
| `0xE1` OK | D→H | `[request_type u8]` |
| `0xE2` ERROR | D→H | `[request_type u8][code u32][utf8 message]` |
| `0xE3` DEVICE_INFO | D→H | `[device_type u32][utf8 name]` |

`axis` is `0` = M1, `1` = M2.

## Build & flash

```
idf.py set-target esp32s3
idf.py build flash monitor
```

Then open the MCP266 console web app and Connect (WebUSB or Web Serial). Click
**Start node**, tick **Live status**, then configure a loop and command a move.
