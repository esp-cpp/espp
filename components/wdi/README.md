# WDI (Wheelchair Digital Interface) Component

`espp::wdi` implements the [Open-Mobility-Hub **Wheelchair HID**
specification](https://open-mobility-hub.github.io/wheelchair-digital-interface/)
(v3.2) — a standard bidirectional interface between a powered wheelchair and an
app / accessory over **USB** or **Bluetooth LE**. It lets an accessory (special
switches, an alternative joystick, a phone app, a companion MCU) drive the chair
and receive status/telemetry back.

The component is layered so the same protocol serves every combination:

- **Protocol core** (`include/detail/wdi_protocol.hpp`) — host-testable, ESP-free:
  the five HID reports, their bitfields, and pack/parse helpers.
- **HID report descriptor** (`include/wdi_hid.hpp`) — the vendor (usage page
  0xFF00) report descriptor, built with the espp `hid-rp` component. It is used by
  **both** transports: the USB HID interface embeds it, and the BLE profile serves
  the identical bytes through its HID-over-GATT Report Map characteristic
  (`10A50002`). Kept out of the dependency-free core so a protocol-only user need
  not pull in `hid-rp`.
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

`ManufacturerId`, the keepalive timing constants, and the BLE GATT UUIDs (service
`10A50001-C4EA-4B47-AE30-A7D9577FC3F9`; HID-over-GATT descriptor characteristics
`10A5000{2..5}` = Report Map / HID Information / HID Control Point / Protocol Mode;
report characteristics `10A5000{6..A}`) are all in the headers.

## Component dependencies

The component itself only `REQUIRES base_component` — the protocol core, `WdiDevice`
and `WdiHost` need nothing else. The **transport** headers are opt-in and pull in
their own dependencies, so a project that includes one must add that dependency to
its own `REQUIRES` (the examples show this):

| Header | Role | Extra dependencies |
|--------|------|--------------------|
| `wdi_hid.hpp` | HID report descriptor | `hid-rp` |
| `wdi_usb.hpp` | USB device (`WdiUsbPeripheral`) | `usb_device`, `hid-rp` |
| `wdi_ble.hpp` | BLE peripheral (`WdiBlePeripheral`) | `esp-nimble-cpp` (+ `hid-rp`, for the Report Map) |
| `wdi_usb_host.hpp` | USB host (`WdiUsbHost`) — *host role, follow-up PR* | `usb_host`, `hid-rp` |
| `wdi_ble_central.hpp` | BLE central (`WdiBleCentral`) — *host role, follow-up PR* | `esp-nimble-cpp` |

This keeps a project that only wants the protocol core (or a single transport)
from pulling in the BLE and USB stacks it does not use.

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

## Device role (`espp::WdiDevice`)

`WdiDevice` (in `wdi.hpp`) is the app / accessory side, transport-agnostic: give
it a `send` callback (put a report on the wire) and feed it the host's reports via
`handle_output()`. It owns the keepalive state machine — call `poll()` periodically
(from an `espp::Timer` / `Task` on device) and it emits a Keepalive when one is due;
`send_control()` / `request_feedback()` reset that timer per the spec. Time is read
through a caller-supplied clock (default: a steady ms clock) so it is fully
host-testable.

```cpp
espp::WdiDevice::Config cfg;
cfg.send = [&](wdi::ReportId id, std::span<const uint8_t> body) {
  return usb.write_hid_report(static_cast<uint8_t>(id), body); // USB HID Input report
};
cfg.on_feedback = [](const wdi::FeedbackReport &f) { /* update UI */ };
espp::WdiDevice dev(cfg);
// app loop / timer:
dev.send_control(joystick_report); // drive the chair
dev.poll();                        // keepalive if due
// transport RX (HID OUT / BLE write): dev.handle_output(id, bytes);
```

### BLE peripheral (`espp::WdiBlePeripheral`)

`wdi_ble.hpp` wraps `WdiDevice` with the WDI GATT service (service `10A50001-…`,
characteristics `10A5000{6..A}`) on `espp::BleGattServer` (esp-nimble-cpp). After
`BleGattServer::init()`, create the service, start it, advertise, and poll:

```cpp
espp::WdiBlePeripheral wdi({.on_feedback = [](const espp::wdi::FeedbackReport &f){ /*...*/ }});
espp::BleGattServer ble;
ble.init("espp WDI");
wdi.make_service(ble.server());
ble.start_services();
wdi.start();
ble.start();
espp::BleGattServer::AdvertisedData adv;
adv.setName("espp WDI");
adv.addServiceUUID(espp::WdiBlePeripheral::service_uuid());
ble.set_advertisement_data(adv);
ble.start_advertising();
// loop: wdi.send_control(report); wdi.poll();  // poll() sends keepalives when due
```

See `ble_example/` for a full runnable example (esp32s3). Control /
Request-Feedback / Keepalive are Notify characteristics (device→central);
Feedback / Keepalive-Response are Write-Without-Response (central→device).

### USB HID device (`espp::WdiUsbPeripheral`)

`wdi_usb.hpp` wraps `WdiDevice` with an `espp::UsbDevice` HID interface using the
WDI report descriptor (`wdi_hid.hpp`). Control / Request-Feedback / Keepalive are
HID **Input** reports (device→host, `write_hid_report()`); Feedback /
Keepalive-Response are HID **Output** reports (host→device, delivered via
`HidFunction::on_receive` — hence `has_out_endpoint`).

```cpp
espp::WdiUsbPeripheral wdi({.on_feedback = [](const espp::wdi::FeedbackReport &f){ /*...*/ }});
std::error_code ec;
wdi.initialize(ec);
// loop: wdi.send_control(report); wdi.poll();  // poll() sends keepalives when due
```

See `usb_example/` for a full runnable example (esp32s3). Because the native USB
port is given to TinyUSB, the console runs on UART0 (with USB-Serial-JTAG as an
early-boot secondary).

## Status

- [x] Protocol core + host tests (`test/wdi_protocol_host_test.cpp`)
- [x] Device role core — `WdiDevice`, keepalive state machine, host-tested
      (`test/wdi_device_host_test.cpp`)
- [x] Device role — **BLE peripheral** (`WdiBlePeripheral`, `wdi_ble.hpp`): the WDI
      GATT service + characteristics on `ble_gatt_server`, with a `ble_example`
- [x] Device role — **USB HID device** (`WdiUsbPeripheral`, `wdi_usb.hpp`): the WDI
      HID report descriptor on `espp::UsbDevice`, with a `usb_example`
- [ ] Host role — USB Host HID + BLE central

## Testing

The protocol core and device role build and run on a host with just a C++20
standard library:

```bash
c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
    components/wdi/test/wdi_protocol_host_test.cpp -o wdi_test && ./wdi_test
c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
    components/wdi/test/wdi_device_host_test.cpp -o wdi_dev_test && ./wdi_dev_test
```

The hid-rp report descriptor also builds on a host (hid-rp is header-only; add it
as `-isystem` so its third-party headers don't trip `-Werror`):

```bash
c++ -std=c++20 -Wall -Wextra -Werror -I components/wdi/include \
    -isystem components/hid-rp/include -isystem components/hid-rp/detail/hid-rp/hid-rp \
    components/wdi/test/wdi_hid_host_test.cpp -o wdi_hid_test && ./wdi_hid_test
```

## Host library (C++ and Python)

The protocol core is bundled into the espp **host library** (`lib/`), so it is
available off-device for CI/interop testing and for building the **WDI host** (the
wheelchair side) on a PC to test a real peripheral against:

- **C++**: the `wdi/include` headers are on the host library's include path
  (`espp::wdi::ControlReport`, `FeedbackReport`, `HostUuid`, `WdiDevice`, …).
- **Python**: `espp.wdi` exposes the reports/bitfields/enums
  (`ControlReport`/`FeedbackReport`/`HostUuid` with `serialize()` / `parse()`),
  so a host or an interop test parses Control reports and builds Feedback reports:

  ```python
  import espp
  wdi = espp.wdi
  got = wdi.ControlReport.parse(bytes_from_peripheral)  # the wheelchair reads control
  fb = wdi.FeedbackReport(); fb.set(wdi.FeedbackBit.DriveEnabled); fb.speed = 4
  send(fb.serialize())                                    # ...and replies with status
  ```

  Python binding test: `python/wdi_test.py`.

## Emulation / safety note

This component can **emulate** a WDI device or host for development and testing.
A powered wheelchair is safety-critical: do not connect an emulator to a real
chair without the manufacturer's guidance, and observe the spec's keepalive /
drive-disable semantics (a lost link must drop to a safe, stopped state).
