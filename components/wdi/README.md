# WDI (Wheelchair Digital Interface) Component

`espp::wdi` implements the [Open-Mobility-Hub **Wheelchair HID**
specification](https://open-mobility-hub.github.io/wheelchair-digital-interface/)
(v3.2) — a standard bidirectional interface between a powered wheelchair and an
app / accessory over **USB** or **Bluetooth LE**. It lets an accessory (special
switches, an alternative joystick, a phone app, a companion MCU) drive the chair
and receive status/telemetry back.

The component is layered so the same protocol serves every combination:

- **Protocol core** (`include/detail/wdi_protocol.hpp`) — host-testable, ESP-free:
  the five HID reports, their bitfields, the shared HID report descriptor, and
  pack/parse helpers.
- **Device role** — the app / accessory: a USB HID **device** (via
  `espp::UsbDevice`) or a BLE **peripheral**. Sends Control, receives Feedback.
- **Host role** — the wheelchair: a USB **host** (USB Host HID) or a BLE
  **central**. Receives Control, sends Feedback.

## Roles and direction

Report directions are named from the **device** (app/accessory) point of view —
an *Input* report is device→host, an *Output* report is host→device:

| Report | ID | Dir | Size | Purpose |
|--------|----|-----|------|---------|
| Control | 0x01 | app→host (Input) | 18 B | joystick X/Y + control-flag bitfields |
| Feedback | 0x02 | host→app (Output) | 19 B | status flags + speed / velocity / odometer |
| Request Feedback | 0x03 | app→host (Input) | 1 B | poll for a Feedback report (`0x01`) |
| Keepalive | 0x04 | app→host (Input) | 1 B | connection heartbeat (`0x01`) |
| Keepalive Response | 0x05 | host→app (Output) | 16 B | the host's 128-bit UUID (manufacturer id + random) |

All report payloads are little-endian **except** the Host UUID, which is
big-endian (network byte order) per the spec.

- **Control** carries an SInt8 `x` (−127 left … +127 right) and `y` (−127 forward
  … +127 reverse) plus four u32 bitfields (Standard1/2, VendorSpecific1/2). A
  `Modifier` bit reverses the seating actuators (e.g. `Tilt | Modifier` = tilt
  back); an all-zero report is a "release".
- **Feedback** carries a u32 status bitfield, two vendor u32s, packed
  speed/profile and velocity nibbles, and an odometer byte.
- **Keepalive**: the app sends a Control / Request-Feedback / Keepalive report
  every ~233 ms; the host disconnects and drive-disables after 3 consecutive
  257 ms windows with no report.

`ManufacturerId`, the keepalive timing constants, and the BLE GATT UUIDs
(service `10A50001-C4EA-4B47-AE30-A7D9577FC3F9`, characteristics `10A5000{6..A}`)
are all in the header.

## Usage (protocol core)

```cpp
#include "detail/wdi_protocol.hpp"
namespace wdi = espp::wdi;

// Build + serialize a Control report (accessory -> wheelchair):
wdi::ControlReport c;
c.x = 0; c.y = -100;                 // forward
c.set(wdi::ControlBit::DriveEnable);
c.set(wdi::ControlBit::SpeedUp);
std::array<uint8_t, wdi::kControlSize> payload = c.serialize();

// Parse a Feedback report (wheelchair -> accessory):
if (auto fb = wdi::FeedbackReport::parse(bytes)) {
  bool moving_ok = fb->has(wdi::FeedbackBit::DriveEnabled);
  float mph = fb->velocity_mph();
}
```

## Status

- [x] Protocol core + host tests (`test/wdi_protocol_host_test.cpp`)
- [ ] Device role — USB HID device (`espp::UsbDevice`) + BLE peripheral, with the
      keepalive state machine
- [ ] Host role — USB Host HID + BLE central
- [ ] Examples (USB + BLE)

## Testing

The protocol core builds and runs on a host with just a C++20 standard library:

```bash
c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
    components/wdi/test/wdi_protocol_host_test.cpp -o wdi_test && ./wdi_test
```

## Emulation / safety note

This component can **emulate** a WDI device or host for development and testing.
A powered wheelchair is safety-critical: do not connect an emulator to a real
chair without the manufacturer's guidance, and observe the spec's keepalive /
drive-disable semantics (a lost link must drop to a safe, stopped state).
