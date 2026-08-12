# Switch 2 Pro Controller (BLE)

Emulate a **Nintendo Switch 2 Pro Controller over BLE** so a real Switch 2
console accepts it as a native controller — including waking the console from
sleep. Built on `espp::BleGattServer` (NimBLE).

> **Status: early — GATT + pairing skeleton.** This milestone stands up the
> custom Nintendo GATT service tree, advertises with Nintendo manufacturer
> data, and implements the reverse-engineered pairing crypto (verified against
> a known-answer vector). The full console init/calibration sequence, input
> report streaming, and console-accept verification on real hardware are
> follow-up milestones (see `DESIGN.md`).

Unlike the original Switch (Bluetooth Classic HID), the Switch 2 uses a
**proprietary BLE GATT interface — not HID-over-GATT — and a custom pairing
handshake, not BLE SMP**. So this component does *not* use espp's `hid_service`
/ `hid-rp`; it builds the Nintendo custom services directly on `BleGattServer`.

```cpp
#include "switch2_pro.hpp"

espp::Switch2Pro controller({.device_name = "Pro Controller"});
controller.init();   // verifies pairing crypto, builds GATT, advertises
```

## The 5 ms connection interval

The console drives the BLE link at a **5 ms** connection interval, below the
7.5 ms Bluetooth spec minimum. Stable input streaming requires the stack to
accept it:

- **ESP32-C6 / C61 / C2 / H2** (RISC-V, open NimBLE controller): enable the
  Kconfig option **`SWITCH2_PRO_PATCH_NIMBLE_5MS`** (default off). When on, the
  component build runs `tools/patch_nimble_5ms.py`, which **binary-patches the
  prebuilt `libble_app.a` in your global `$IDF_PATH` install** to lower the
  minimum-interval floor. This modifies your ESP-IDF installation; undo with
  `python tools/patch_nimble_5ms.py --target esp32c6 --restore`.
- **ESP32-S3 / C3**: a different closed controller; the equivalent is an
  Espressif Kconfig path (esp-idf#18467), not this binary patch. See `DESIGN.md`.

The patch is **not** required for pairing or the GATT skeleton — only for
stable input streaming — so it stays off by default.

## Attribution

The protocol was reverse-engineered by the community, principally
[ndeadly/switch2_controller_research](https://github.com/ndeadly/switch2_controller_research).
The overall approach and the NimBLE-patch technique are adapted (MIT) from
[zhantss/ESP32-BLE5-NSController-Emulator](https://github.com/zhantss/ESP32-BLE5-NSController-Emulator).
This component reimplements the interoperability protocol on espp/NimBLE; it
contains no Nintendo or Espressif binaries. The pairing "authentication" relies
on a published fixed key and is a possession check, not per-device attestation.

## Example

See [example](./example) — builds for ESP32-C6 (primary) and is buildable for
S3. It brings up the controller, runs the pairing-crypto self-test, and
advertises for a console to pair with.
