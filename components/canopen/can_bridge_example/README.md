# USB &lt;-&gt; CAN Bridge Example

Turns an ESP32-S3 into a **WebUSB / Web Serial CAN interface**: the hosted
[CAN bridge console web app](https://esp-cpp.github.io/espp/apps/can_bridge_console.html)
connects over the native USB and can

- **send** CAN frames as a normal, ACK-ing bus participant ("master"), and
- **inspect** the bus — stream every received frame; in *listen-only* mode the
  node is a passive sniffer that never ACKs or transmits.

It bridges the ESP32-S3 TWAI (CAN 2.0) controller to the host over USB using the
espp `stream_frame` framing and an `espp::Dispatcher` (this example uses
**module id 5** by default — `kCanBridgeModule` at the top of
`can_bridge_example.cpp` is the one place to change it; the hosted consoles
find it through discovery by its protocol id `espp.can-bridge`). The same
framed protocol is exposed
on both the USB **vendor**
interface (WebUSB) and a **CDC** interface (Web Serial), so the web app can use
either transport. The system console/logs go to **UART0** (set in
`sdkconfig.defaults`): on the S3 / P4, USB-Serial-JTAG shares the native USB
port's PHY with USB-OTG, which TinyUSB takes over.

## Wiring

Connect the TWAI TX/RX GPIOs to a CAN transceiver (e.g. SN65HVD230, TJA1050) on
a properly terminated (120 Ω) bus. Defaults (change in `can_bridge_example.cpp`):

| Signal | GPIO |
|--------|------|
| TWAI TX | 17 |
| TWAI RX | 16 |

Listen-only mode monitors an existing bus without a transceiver ACKing, but a
transceiver is still required to receive the differential signal.

## Protocol (module 5)

Framed with `stream_frame` and routed by `espp::Dispatcher`. The full base
header order on the wire (all multi-byte fields little-endian) is:

```
[magic u16 "OT"][flags u8][module u8][type u8][len u32][payload…][crc32 u32]
```

The base header is 9 bytes. `module` is **5** for this bridge. `flags` bit0 =
reply (0 = host→device request, 1 = device→host reply/event), bits 4-7 =
version = 1 — so a request `flags` byte is `0x10` and a reply is `0x11`. (v2
also defines an optional correlation-id field gated by `flags` bit1, inserted
between `type` and `len`; the CAN bridge never sets it, so its frames always use
the 9-byte base header.) `crc32` covers the header + payload. Host→device
requests use type high-nibble 5; device→host replies/events use high-nibble D.

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

A CAN frame is encoded as `[id u32][flags u8][dlc u8]` optionally followed by
`dlc` data bytes, where `flags` bit0 = extended (29-bit) and bit1 = RTR. The
data bytes are present **only for non-RTR frames**: an RTR frame is just the
6-byte header even when its `dlc` is nonzero (the DLC is the requested response
length, not a data length). A client must therefore append no data for RTR
frames — the bridge encodes and expects none — so the payload is 6 bytes for RTR
and `6 + dlc` (6..14) otherwise.

The bus starts **stopped**: the host sets baudrate/mode with `SET_CONFIG`, then
`START`. `SET_CONFIG` is rejected while the bus is running (stop first).

## Build & flash

```
idf.py set-target esp32s3
idf.py build flash monitor   # console is on UART0 (USB-UART adapter)
```

Then open the CAN console web app and Connect (WebUSB or Web Serial).

## Simulated CANopen node (no CAN hardware)

To try the [CAN console](https://esp-cpp.github.io/espp/apps/can_bridge_console.html)
and the [DS402 panel](https://esp-cpp.github.io/espp/apps/ds402_panel.html)
without a transceiver, bus or drive, build the bridge with a **simulated
CANopen CiA 402 node** in place of the TWAI peripheral. It is off by default;
enable it in `idf.py menuconfig` under *CAN Bridge Example Configuration*, or
build with the extra defaults file:

```
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.simulated" build flash
```

The USB side and the bridge protocol are unchanged, so the web apps connect and
work exactly as with real hardware (set a baudrate, START the bus, then talk to
node id 1 — `CONFIG_CAN_BRIDGE_SIMULATED_NODE_ID`). Frames the host sends are
answered by the node in firmware (`main/simulated_ds402_node.hpp`) instead of
going out on a bus, and the node's own traffic streams back as `CAN_RX`:

- **NMT** start / stop / pre-operational / reset node / reset communication,
  the boot-up message, and the producer heartbeat (`0x1017`, 1 s by default).
- **SDO** server on `0x600`/`0x580` + id: expedited and segmented upload and
  download (toggle bit checked) with the CiA 301 abort codes (unknown object /
  sub-index, read-only, length mismatch, value range, ...).
- An **object dictionary** with the CiA 301 communication and identity objects
  (`0x1000`, `0x1001`, `0x1008`–`0x100A`, `0x1010`/`0x1011` with the
  `save`/`load` signatures, `0x1017`, `0x1018`, `0x1200`, PDO parameters), its
  own **stored EDS** (`0x1021`, a DOMAIN generated from the dictionary; `0x1022`
  = 0) so a browser can read the device's object list from the device, the
  CiA 402 objects of a single-axis drive, and manufacturer objects: write `1`
  to `0x2000` to inject a fault (an EMCY is sent; clear it with the controlword
  fault-reset edge).
- The **DS402 state machine** driven by the controlword (`0x6040`) and reported
  in the statusword (`0x6041`): the enable sequence, disable / shutdown,
  quick-stop (transits to Switch On Disabled once stopped, `0x605A` = 2) and
  fault reset; the supported **modes** (`0x6502`) are profile position (with
  the new-set-point handshake, absolute / relative, halt), profile velocity
  (ramping at the profile acceleration / deceleration), profile torque and
  homing. Position / velocity / torque actual values (`0x6064`, `0x606C`,
  `0x6077`) follow a simple trapezoidal motion model.
- **TPDO1** (statusword + position actual) every `0x1800:5` ms (100 by
  default) while Operational (NMT start), also on an RTR; **RPDO1**
  (controlword + modes of operation) is applied.

The node is host-buildable and unit-tested against the `canopen` component's
client-side frame builders / parsers:

```
cd test && c++ -std=c++20 -I../../include -I../main simulated_node_host_test.cpp -o test && ./test
```

The simulation has no bit timing, so the baudrate is only reported; in
listen-only mode the bridge refuses to transmit (as a real listen-only node
cannot) while the node's heartbeat and boot-up traffic is still observed.
