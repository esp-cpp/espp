# Switch 2 Pro Controller (BLE)

Emulate a **Nintendo Switch 2 Pro Controller over BLE** so a real Switch 2
console accepts it as a native controller — including waking the console from
sleep. Built on `espp::BleGattServer` (NimBLE).

> **Status: fully working on the ESP32-C6 (recommended target).**
> Verified against a real Switch 2 (ESP32-C6-DevKit): the console pairs with and
> accepts the emulator as a Pro Controller (full battery, correct icon), input
> streams **continuously and lag-free** (one report per live connection interval,
> matching a real controller — ~62 Hz during the initial 15 ms window, then ~200 Hz
> once the console moves the link to 5 ms; see "The 5 ms connection interval"),
> buttons and sticks register on the console's "Test Input Devices" screen, and
> **reconnect**
> (including the console reconnecting on its own after a reboot) and
> **wake-from-sleep** both work without re-pairing. What's implemented:
> advertising with Nintendo manufacturer data, the exact GATT handle layout,
> the reverse-engineered pairing crypto (known-answer verified) + LTK injection
> for standard LL encryption, the full console init/command sequence, bond
> persistence (NVS), and continuous input-report streaming with real
> backpressure.
>
> **ESP32-S3: also fully working on ESP-IDF ≥ v6.1 (verified on real hardware).**
> The S3's sub-spec-interval rejection (which blocked reconnect/wake) is fixed by
> ESP-IDF's `CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE` (default on;
> espressif/esp-idf#18467), backported to v6.0/v5.5/v5.4/v5.3 — so on a current
> IDF the S3 needs **no binary patch**. Verified against a real Switch 2 on
> **v6.1**: pairing, continuous lag-free input, reconnect, and wake-from-sleep all
> work. The earlier finding that the *closed BTDM controller stalls sustained tx
> ~3 s into a stream* was on the **pre-fix (v6.0.1) controller lib**; v6.1 ships an
> updated controller lib and the stall is gone. See "The 5 ms connection interval"
> and "Known issues".

Unlike the original Switch (Bluetooth Classic HID), the Switch 2 uses a
**proprietary BLE GATT interface — not HID-over-GATT — and a custom pairing
handshake, not BLE SMP**. So this component does *not* use espp's `hid_service`
/ `hid-rp`; it builds the Nintendo custom services directly on `BleGattServer`.

```cpp
#include "switch2_pro.hpp"

espp::Switch2Pro controller({.device_name = "Pro Controller"});
controller.init();   // verifies pairing crypto, builds GATT, advertises
```

## The 5 ms connection interval (required for sustained input, reconnect & wake)

The console chooses the connection interval **in its `CONNECT_IND`, based on
whether it recognises the controller**:

- **Fresh pairing:** the `CONNECT_IND` is 15 ms — spec-legal, works on a stock
  controller — and the 0x15 pairing handshake completes at 15 ms. But **~1.5 s
  after input subscription the console sends an `LL_CONNECTION_UPDATE` dropping
  the link to 5 ms** for the rest of the session (observed on real S3 hardware:
  `CONN PARAMS UPDATE: itvl=5.00ms` right after streaming begins). So pairing
  itself works unpatched, but *sustained* first-session input needs 5 ms support
  too — on a stock controller the stream dies seconds in, when the console makes
  that switch. (This is the real cause of the old "~3 s tx-stall"; see "Known
  issues".)
- **Reconnect / wake (bonded controller):** `CONNECT_IND interval=4` — **5 ms
  from the very first packet**, below the 7.5 ms Bluetooth spec minimum
  (verified in both the reconnect and wake captures, and on real hardware). A
  stock controller cannot accept that connection, so reconnect/wake silently
  fail: the console wakes on our advertisement, attempts the 5 ms connection, and
  gives up.

Either way the console ends up driving the link at 5 ms, so accepting the
sub-spec interval is required for anything past the initial pairing handshake.

Sustained input in every mode — plus reconnect and wake-from-sleep — therefore
requires the controller (and the BLE host) to accept that sub-spec interval. Only
the initial pairing handshake (the first ~1.5 s, before the console's switch to
5 ms) works without it. There are two ways to enable it, by chip:

### ESP32-S3 / C3 — official ESP-IDF option (recommended)

ESP-IDF now ships an **official** controller option that lets the S3/C3 BTDM
controller — and the BLE host — accept connection intervals below the 7.5 ms
spec minimum (down to 3.75 ms, which covers the console's 5 ms), no binary
patching required:

```
CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE=y   # default y
```

It `select`s `BT_BLE_HOST_ALLOW_SUB_SPEC_MIN_CONN_INT`, so the NimBLE host
accepts the sub-spec interval too. It is **on by default**, so on a recent IDF
the S3 accepts the console's 5 ms reconnect/wake out of the box.

Requires an ESP-IDF with the fix (espressif/esp-idf#18467), backported to
**v6.0 (≥ `142aea3`), v5.5 (≥ `cf13345`), v5.4 (≥ `aefcf1c`), v5.3
(≥ `9831261`)**, and confirmed on **v6.1**. On an older IDF the option does not
exist — **update to a fixed IDF** (there is no verified binary-patch fallback for
S3/C3; see the note below).

> Verified on real hardware with **ESP-IDF v6.1**: with this option default-on,
> the S3 pairs, streams input lag-free, reconnects, and wakes the console with no
> binary patch.
>
> **No S3/C3 binary-patch fallback.** A patch of the pre-fix BTDM controller
> (`libbtdm_app.a`, `r_llc_con_upd_param_in_range`) was reverse-engineered but never
> confirmed to enable 5 ms on hardware — patching that min-interval compare alone is
> reported insufficient on the pre-fix S3 (esp-idf#18467), matching our own testing
> where it had no effect. So `SWITCH2_PRO_PATCH_NIMBLE_5MS` covers only the
> C6-family chips below; on S3/C3 use the official option above (update your IDF).

### ESP32-C6 / C61 / C2 / H2 — binary patch

The open NimBLE controller (`libble_app.a`) has no equivalent config option yet
(Espressif support is planned), so these chips use the opt-in Kconfig option
**`SWITCH2_PRO_PATCH_NIMBLE_5MS`** (off by default — it mutates the prebuilt
controller lib in your global `$IDF_PATH`, which is too invasive to do silently).
When enabled, the build runs `tools/patch_nimble_5ms.py`, which binary-patches the
minimum-interval floor from 6 units (7.5 ms) to 4 (5 ms):

- **ESP32-C6 / C61 / C2 / H2** (RISC-V, open NimBLE controller): `libble_app.a`,
  `addi a5,a4,-6` → `-4`.

(The patcher only supports these targets; it refuses S3/C3 — use the official
option above there.)

This modifies your ESP-IDF installation; undo with `python
tools/patch_nimble_5ms.py --target <chip> --restore`, and check the current state
(no hardware needed) with `python tools/smoke_test_5ms.py --target <chip>`, which
disassembles the controller and reports whether 5 ms is accepted or rejected.

## Stability notes & known issues

**ESP32-S3: the closed BTDM BLE controller stalled sustained notification tx — fixed in ESP-IDF v6.1.**

> **Resolved.** The analysis below was done on ESP-IDF **v6.0.1**, whose S3
> controller lib predates the official sub-spec-interval fix
> (`CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`, espressif/esp-idf#18467). **v6.1
> ships the updated controller lib and the stall is gone** — verified on real
> hardware: the S3 now streams input to a real Switch 2 continuously and lag-free
> and holds reconnect/wake, matching the C6. The description below is retained as
> the pre-fix (v6.0.1) state for anyone stuck on an older IDF.

The issue isolated by elimination on real hardware: ~3 s into
any sustained encrypted notification stream, the S3's controller stops
servicing tx — completions become sporadic (hundreds of ms apart), input dies,
and the link eventually supervision-times-out. Everything host-side is
demonstrably healthy at that moment (mbuf pools near-full, controller ACL
buffers free, host task responsive), and the stall time was **independent of the
send rate** (62 Hz and 31 Hz stalled at the same wall-clock, ruling out
per-packet resource exhaustion). The identical firmware on an **ESP32-C6**
(open-source NimBLE controller) streamed indefinitely with zero distress.

**The v6.1 log revealed the mechanism.** The stall was not a random controller
fault — it lined up exactly with the console's behaviour now visible on a working
S3: ~1.5 s after input subscription the console sends an `LL_CONNECTION_UPDATE`
dropping the link from 15 ms to **5 ms** (`CONN PARAMS UPDATE: itvl=5.00ms`). The
pre-fix S3 BTDM controller could not apply that sub-spec interval, so tx servicing
collapsed a few seconds into every session — which is why the "stall" always hit
at the same wall-clock regardless of send rate (it was the console's timed switch,
not resource exhaustion), and why the C6 (which accepts 5 ms via the binary patch)
never showed it. It is the **same** sub-spec-interval limitation as reconnect/wake,
just arriving mid-session. Espressif's controller-lib update in v6.1 (with
`CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`) makes the S3 accept the 5 ms update,
and the stream now holds — verified on real hardware. On a pre-fix IDF, on-change
streaming (`Config::continuous_streaming = false`) reduces traffic enough to
partially mask it (input works in bursts); the real fix is updating to
ESP-IDF ≥ v6.1 (or a C6-class chip).

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

**Set `CONFIG_FREERTOS_HZ=1000` (required for accurate stream cadence).** The
streaming task paces itself with sub-15 ms sleeps (down to ~5 ms once the console
moves the link there), and `std::this_thread::sleep_for` is quantised to the
FreeRTOS tick. At the 100 Hz default a 5 ms sleep rounds to ~10 ms and 15 ms to
~20 ms, desyncing from the connection interval. The example sets this; a consuming
project must set it too (the component logs a warning at init if it is lower).

**What the captures established** (ndeadly's `nrf52840` captures, decrypted,
via `tshark`): a real controller's fresh pairing's `CONNECT_IND` is **15 ms**
and its reconnect **and wake** connect at **5 ms**. Active-use motion captures
run at ~7.5 ms / 133 Hz for minutes. **On real hardware the console does not stay
at the fresh-pair 15 ms:** ~1.5 s after input subscription it sends an
`LL_CONNECTION_UPDATE` dropping the link to 5 ms and drives the session there
(the only other post-connect LL change is the PHY switch to 2M). So the console
ends up at 5 ms in *every* mode — fresh session (after a brief 15 ms window),
reconnect, and wake — which is why sub-spec-interval support is required for
sustained input, not just for reconnect/wake (see "The 5 ms connection interval"
above). Advertising variants: the bonded
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

See [example](./example) — default target ESP32-C6 (also fully working on the
ESP32-S3 with ESP-IDF ≥ v6.1). It brings up the controller, runs the
pairing-crypto self-test, advertises for a console to pair with, and maps the
BOOT button to A (GPIO0 on Xtensa boards, GPIO9 on the RISC-V devkits).
