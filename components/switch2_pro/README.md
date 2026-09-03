# Switch 2 Pro Controller (BLE)

Emulate a **Nintendo Switch 2 Pro Controller over BLE** so a real Switch 2
console accepts it as a native controller — including waking the console from
sleep. Built on `espp::BleGattServer` (NimBLE).

> **Status: fully working on the ESP32-C6 (recommended target).**
> Verified against a real Switch 2 (ESP32-C6-DevKit): the console pairs with and
> accepts the emulator as a Pro Controller (full battery, correct icon), input
> streams **continuously and lag-free** (one report per 15 ms connection
> interval, ~62 Hz — matching a real controller's cadence), buttons and sticks
> register on the console's "Test Input Devices" screen, and **reconnect**
> (including the console reconnecting on its own after a reboot) and
> **wake-from-sleep** both work without re-pairing. What's implemented:
> advertising with Nintendo manufacturer data, the exact GATT handle layout,
> the reverse-engineered pairing crypto (known-answer verified) + LTK injection
> for standard LL encryption, the full console init/command sequence, bond
> persistence (NVS), and continuous input-report streaming with real
> backpressure.
>
> **ESP32-S3: pairs, but input only works in short bursts.** The S3's *closed*
> BTDM BLE controller stops servicing notification tx ~3 s into any sustained
> encrypted stream (rate-independent; host and buffers healthy) and the link
> then supervision-times-out — see "Known issues" below. Use a chip from the
> open-NimBLE-controller family (C6/C61/C2/H2) instead.

Unlike the original Switch (Bluetooth Classic HID), the Switch 2 uses a
**proprietary BLE GATT interface — not HID-over-GATT — and a custom pairing
handshake, not BLE SMP**. So this component does *not* use espp's `hid_service`
/ `hid-rp`; it builds the Nintendo custom services directly on `BleGattServer`.

```cpp
#include "switch2_pro.hpp"

espp::Switch2Pro controller({.device_name = "Pro Controller"});
controller.init();   // verifies pairing crypto, builds GATT, advertises
```

## The 5 ms connection interval (required for reconnect & wake)

The console chooses the connection interval **in its `CONNECT_IND`, based on
whether it recognises the controller**:

- **Fresh pairing:** 15 ms — spec-legal, works on a stock controller. The
  console then services continuous 62 Hz input at 15 ms indefinitely (it never
  renegotiates mid-session), which is why pairing and first-session input work
  unpatched.
- **Reconnect / wake (bonded controller):** `CONNECT_IND interval=4` — **5 ms
  from the very first packet**, below the 7.5 ms Bluetooth spec minimum
  (verified in both the reconnect and wake captures). A stock controller
  cannot accept that connection, so reconnect/wake silently fail: the console
  wakes on our advertisement, attempts the 5 ms connection, and gives up.

So the controller patch is **required for reconnect and wake-from-sleep** — but
it is **off by default**, because enabling it mutates the prebuilt controller
library in your global `$IDF_PATH`, which is too invasive to do silently. Fresh
pairing and first-session input work without it; enable it (uncomment
`CONFIG_SWITCH2_PRO_PATCH_NIMBLE_5MS` in the example's `sdkconfig.defaults`, or
run the tool directly) to get reconnect/wake. The Kconfig option
**`SWITCH2_PRO_PATCH_NIMBLE_5MS`** makes the component build run
`tools/patch_nimble_5ms.py`, which **binary-patches the prebuilt controller
library** to lower the minimum-interval floor from 6 units (7.5 ms) to 4 (5 ms):

- **ESP32-C6 / C61 / C2 / H2** (RISC-V, open NimBLE controller): `libble_app.a`,
  `addi a5,a4,-6` → `-4`.
- **ESP32-S3 / C3** (BTDM / RivieraWaves controller): `libbtdm_app.a` (+ the
  `_flash` variant), `r_llc_con_upd_param_in_range` — S3 `bltui a4,6` → `bltui
  a4,4`, C3 `li a6,5` → `li a6,3`.

This modifies your ESP-IDF installation; undo with `python
tools/patch_nimble_5ms.py --target <chip> --restore`, and check the current state
(no hardware needed) with `python tools/smoke_test_5ms.py --target <chip>`, which
disassembles the controller and reports whether 5 ms is accepted or rejected.

## Stability notes & known issues

**ESP32-S3: the closed BTDM BLE controller stalls sustained notification tx.**
The one unresolved issue, isolated by elimination on real hardware: ~3 s into
any sustained encrypted notification stream, the S3's controller stops
servicing tx — completions become sporadic (hundreds of ms apart), input dies,
and the link eventually supervision-times-out. Everything host-side is
demonstrably healthy at that moment (mbuf pools near-full, controller ACL
buffers free, host task responsive), and the stall time is **independent of the
send rate** (62 Hz and 31 Hz stall at the same wall-clock, ruling out
per-packet resource exhaustion). The identical firmware on an **ESP32-C6**
(open-source NimBLE controller) streams indefinitely with zero distress — and
reconnect-after-console-reboot, which never worked on the S3, works on the C6.
Conclusion: an S3 BTDM controller bug/incompatibility on this link
configuration (2M PHY + LL encryption + sustained peripheral notifications),
unreachable from the host. On the S3, on-change streaming
(`Config::continuous_streaming = false`) reduces traffic enough to partially
mask it (input works in bursts); the real fix is using a C6-class chip.

Diagnostics that survive in the driver (useful if this ever regresses): a
per-500 ms stream-health debug log (drain rate, backpressure skips, ENOMEM
count, per-pool free/low-water), a one-shot `TX WEDGE` warning pinpointing the
first ENOMEM's origin (host mbuf vs downstream), and an `@disconnect` summary
tying the disconnect to the tx timeline.

**Streaming model.** `set_input_report()` only stores the latest state; a
driver-owned task streams it — by default **continuously**, one report per live
connection interval with the byte-0 counter incrementing every report, matching
a real controller (verified in captures: a real device streams 62 Hz at the
15 ms pairing interval). The task applies **real backpressure** by capping
un-drained host mbufs (the `NOTIFY_TX` event fires at host→controller handoff,
*not* over-air completion, so counting those cannot detect a backlog — the mbuf
pool level can). The 40-byte IMU motion block is sent all-zero by default
(accepted by the console); `Config::stream_imu_motion` replays captured frames
instead.

**Dedicate a core to BLE (dual-core chips).** With everything on core 0 (the
ESP-IDF default), a fast input-notify loop starves the NimBLE host and the link
supervision-times-out within seconds. The example's `sdkconfig.defaults` pins
the BLE controller and host to core 1 on dual-core targets and enlarges the
NimBLE mbuf/ACL pools. Also keep HCI commands (e.g. `ble_gap_read_le_phy`) out
of any per-interval loop — at 62 Hz they flood the HCI path and stall the
host's data tx.

**What the captures established** (ndeadly's `nrf52840` captures, decrypted,
via `tshark`): a real controller's fresh pairing runs at **15 ms** and its
reconnect **and wake** at **5 ms**, fixed in the `CONNECT_IND` and never
renegotiated mid-session (zero `LL_CONNECTION_UPDATE_IND` /
`LL_CONNECTION_PARAM_REQ` in any capture; the only post-connect LL change is
the PHY switch to 2M). Active-use motion captures run at ~7.5 ms / 133 Hz for
minutes. The console services an emulated controller's *fresh-pair* session at
15 ms / 62 Hz indefinitely, but connects to a *bonded* controller only at 5 ms
— which is why the controller patch is required for reconnect/wake (see "The
5 ms connection interval" above). Advertising variants: the bonded
manufacturer-data payload embeds the console's identity address, with flag
byte `0x00` for passive reconnect presence and `0x81` ("user pressed a button
— connect to me") for wake; an idle console ignores the passive variant, so
user-initiated wake should broadcast `0x81` (see `wake_console()`).

## Attribution

The protocol was reverse-engineered by the community, principally
[ndeadly/switch2_controller_research](https://github.com/ndeadly/switch2_controller_research).
The overall approach and the NimBLE-patch technique are adapted (MIT) from
[zhantss/ESP32-BLE5-NSController-Emulator](https://github.com/zhantss/ESP32-BLE5-NSController-Emulator).
This component reimplements the interoperability protocol on espp/NimBLE; it
contains no Nintendo or Espressif binaries. The pairing "authentication" relies
on a published fixed key and is a possession check, not per-device attestation.

## Example

See [example](./example) — default target ESP32-C6 (recommended; also builds
for S3, with the streaming caveat above). It brings up the controller, runs the
pairing-crypto self-test, advertises for a console to pair with, and maps the
BOOT button to A (GPIO0 on Xtensa boards, GPIO9 on the RISC-V devkits).
