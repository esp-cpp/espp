# USB &lt;-&gt; CAN Bridge Example

Turns an ESP32-S3 into a **WebUSB / Web Serial CAN interface**: the hosted
[CAN console web app](https://esp-cpp.github.io/espp/apps/can_console.html)
connects over the native USB and can

- **send** CAN frames as a normal, ACK-ing bus participant ("master"), and
- **inspect** the bus — stream every received frame; in *listen-only* mode the
  node is a passive sniffer that never ACKs or transmits.

It bridges the ESP32-S3 TWAI (CAN 2.0) controller to the host over USB using the
espp `stream_frame` framing and an `espp::Dispatcher` (this example owns
**module id 5**). The same framed protocol is exposed on both the USB **vendor**
interface (WebUSB) and a **CDC** interface (Web Serial), so the web app can use
either transport. The system console/logs stay on the separate built-in
USB-Serial-JTAG.

## Wiring

Connect the TWAI TX/RX GPIOs to a CAN transceiver (e.g. SN65HVD230, TJA1050) on
a properly terminated (120 Ω) bus. Defaults (change in `can_bridge_example.cpp`):

| Signal | GPIO |
|--------|------|
| TWAI TX | 5 |
| TWAI RX | 4 |

Listen-only mode monitors an existing bus without a transceiver ACKing, but a
transceiver is still required to receive the differential signal.

## Protocol (module 5)

Framed with `stream_frame` (magic `OT` / type / len / crc32) and routed by
`espp::Dispatcher`. Host→device requests use type high-nibble 5; device→host
replies/events use high-nibble D.

| Type | Dir | Meaning |
|------|-----|---------|
| `0x50` CAN_TX | H→D | transmit a CAN frame |
| `0x51` SET_CONFIG | H→D | `[baudrate u32][mode u8][rsv u8]` (mode 0=normal, 1=listen-only) |
| `0x52` START | H→D | bring the bus up with the current config |
| `0x53` STOP | H→D | take the bus down |
| `0x54` GET_STATUS | H→D | request a STATUS reply |
| `0xD0` CAN_RX | D→H | a received CAN frame |
| `0xD1` OK | D→H | ack |
| `0xD2` ERROR | D→H | `[code u32][utf8 message]` |
| `0xD3` STATUS | D→H | `[baudrate u32][mode u8][running u8][rx u32][tx u32][err u32]` |

A CAN frame is encoded as `[id u32][flags u8][dlc u8][data: dlc bytes]`, where
`flags` bit0 = extended (29-bit) and bit1 = RTR.

The bus starts **stopped**: the host sets baudrate/mode with `SET_CONFIG`, then
`START`. `SET_CONFIG` is rejected while the bus is running (stop first).

## Build & flash

```
idf.py set-target esp32s3
idf.py build flash monitor
```

Then open the CAN console web app and Connect (WebUSB or Web Serial).
