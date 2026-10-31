# switch2_pro — design notes

## Goal

Emulate a **Nintendo Switch 2 Pro Controller over BLE** so a real Switch 2 console
accepts it as a native controller — including waking the console from sleep over BLE.

This is NOT the Switch 1 protocol. The Switch 2 moved controllers from Bluetooth
Classic HID to **BLE with a proprietary GATT layer** (not HID-over-GATT), a custom
pairing scheme (not BLE SMP), and a custom command channel. So espp's existing
`hid_service` / `hid-rp` (standard HOGP + report descriptors) do **not** apply here;
this component builds custom GATT services directly on `espp::BleGattServer`.

## Sources / prior art

- **Protocol facts**: `ndeadly/switch2_controller_research` (byte-level GATT map,
  pairing handshake, command set, report formats; decrypted sniffer captures).
- **Working ESP32 reference** (MIT): `zhantss/ESP32-BLE5-NSController-Emulator` —
  raw-NimBLE C emulator that a real Switch 2 accepts. We adapt its *approach and
  structure* (with attribution) and reimplement on esp-nimble-cpp / `BleGattServer`.
  We do **not** copy ndeadly's prose/tables wholesale, and we do **not** vendor or
  modify Espressif's controller libraries.

## Feasibility (verified)

Not blocked by cryptographic attestation. The pairing "authentication" is weak and
reproducible: a **fixed controller key** `B1 = 5CF6EE792CDF05E1BA2B6325C41A5F10`, an
XOR-derived link key `LTK = A1 ⊕ B1`, and a single AES-128-ECB possession proof
`B2 = AES_ECB(reverse(LTK), reverse(A2))`. Golden vector (host-verified with openssl):

    A1  = 3503e92982877124bea80c664615834b   (host public key, from console)
    B1  = 5cf6ee792cdf05e1ba2b6325c41a5f10   (fixed controller key)
    A2  = 6fc6df8ad8fedf15bb8c15e91f320544   (host challenge)
    LTK = 69f50750ae5874c504836f43820fdc5b   (= A1 ⊕ B1)
    B2  = 134c97f511b9b6dd4d86fd40f536e9ed   (= AES-128-ECB(rev(LTK), rev(A2)))

`switch2_pro_pairing.*` implements this and self-tests against the golden vector at
init (logged pass/fail) — verifiable on-device with no console.

## The 5 ms connection-interval problem

The console drives the link at a **5 ms** connection interval — below the 7.5 ms BLE
spec minimum. The controller stack must accept it or reconnect/wake won't form.

Two routes by chip family:

**ESP32-S3 / C3 — official ESP-IDF option (the verified path).** ESP-IDF added
`CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE` (default `y`), which lets the BTDM
controller and the BLE host accept sub-spec intervals (down to 3.75 ms) with no
binary patching — in ESP-IDF ≥ v6.0 (`142aea3`) / v5.5 (`cf13345`) / v5.4
(`aefcf1c`) / v5.3 (`9831261`), and confirmed on **v6.1**; see
espressif/esp-idf#18467. The S3 is verified end-to-end on real hardware this way,
and v6.1's updated controller lib also fixes the sustained-tx stall seen on v6.0.1.
There is **no binary-patch fallback for S3/C3**: a patch of the pre-fix BTDM
library (`libbtdm_app.a`, `r_llc_con_upd_param_in_range` — the peripheral-side
connection-parameter validator) was reverse-engineered but never confirmed to
enable 5 ms on hardware (patching that min-interval compare alone is reported
insufficient on the pre-fix S3, esp-idf#18467, matching our own testing where it
had no effect), so it is not shipped. Those RE notes live in git history.

**ESP32-C6 / C61 / C2 / H2 — no supported route today.** The open RISC-V NimBLE
controller has no equivalent config option, and the component does not modify the
ESP-IDF controller libraries (decision: espp ships nothing that mutates a user's
IDF install). The protocol was developed and verified on a C6 with a locally
modified controller, so these chips work as soon as Espressif exposes a
sub-spec-interval option for the NimBLE controller.

## GATT layout (reproduced from captures)

Two proprietary primary services; contiguous handles matter for some console
firmwares (FW 2.0.0+ shifts them +8 for headset audio, so absolute-handle dependence
is not strict — we reproduce the map but discover by UUID).

    00c5af5d-1964-4e30-8f51-1956f96bd280   (svc1, purpose unclear; chars …281/282/283)
    ab7de9be-89fe-49ad-828f-118f09df7fd0   (svc2, main)
      ab7de9be-…-fd2   READ/NOTIFY   common input report (0x05)
      7492866c-…       READ/NOTIFY   Pro Controller 2 input report (0x09)
      cc483f51-…       WRITE_NR      vibration / HD rumble
      649d4ac9-…       WRITE_NR      command (basic)
      3dacbc7e-…       WRITE_NR      vibration+command combined (pairing runs here)
      4147423d-…       WRITE_NR      firmware update (large)
      c765a961-…       NOTIFY        command response #1
      506d9f7d-…       NOTIFY        command response #2

Security: **no SMP** — the console app-level-pairs over the command channel and will
disconnect a peer that initiates SMP. We configure NimBLE not to initiate pairing;
the LTK from the 0x15 exchange is what encrypts the link. Bond (host addr + LTK)
persists in NVS for reconnect + wake.

## Milestones (all implemented; verified end-to-end on ESP32-S3, and on a C6 with a modified controller)

1. **GATT + pairing skeleton**: custom GATT tree stands up, advertises with
   Nintendo manufacturer data, completes the 0x15 pairing handshake (crypto
   known-answer verified) and the console accepts pairing.
2. **Command dispatch + init sequence** (flash/calibration reads, feature-select,
   LEDs, firmware-update-prompt suppression) so the console finishes bring-up.
3. **Input report streaming** (report 0x09: buttons incl. C/GL/GR, 12-bit sticks,
   IMU block) streamed continuously with real backpressure. The `CONNECT_IND` and
   pairing run at 15 ms, but ~1.5 s after subscription the console issues an
   `LL_CONNECTION_UPDATE` dropping the link to **5 ms** for the rest of the session
   (observed on real S3 hardware) — so sustained input needs sub-spec-interval
   support (see milestone 4), not just reconnect/wake.
4. **Reconnect + wake-from-sleep** (bonded reconnect with the 0x81 wake flag).
   The console connects a bonded controller at 5 ms from the first packet, and
   drops even the fresh session to 5 ms mid-stream, so the link runs at 5 ms in
   every mode. On S3/C3 use ESP-IDF ≥ v6.1's official
   `CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE` (default on); the C6-family
   controller has no such option yet (see above).

On ESP-IDF ≥ v6.1 the ESP32-S3 is also fully verified (pairing, continuous input,
reconnect, wake) — its updated BTDM controller lib both accepts the console's 5 ms
CONNECT_IND (via `CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`) and sustains the
encrypted input stream, fixing the ~3 s tx-stall seen on v6.0.1 (see the README
"Known issues").

## Component layout

    switch2_pro/
      include/switch2_pro.hpp            Switch2Pro class (over BleGattServer)
      include/switch2_pro_protocol.hpp   UUIDs, command/subcommand ids, feature bits, fixed key, golden vector
      include/switch2_pro_report.hpp     Pro Controller 2 input report (0x09) packed struct
      src/switch2_pro.cpp                GATT setup, advertising, GAP, command dispatch, pairing state machine, streaming
      src/switch2_pro_pairing.cpp        pairing crypto (mbedTLS PSA) + self-test
      src/switch2_pro_flash.hpp          simulated controller flash (captured calibration / device-info blocks)
      src/switch2_pro_motion.hpp         captured IMU motion frames (optional replay)
      example/                           S3-primary, C6-buildable
