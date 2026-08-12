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
  We do **not** copy ndeadly's prose/tables wholesale, and we do **not** vendor
  Espressif's `libble_app.a`.

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
spec minimum. The controller stack must accept it or the console won't stream input.

- **C6 / C61 / C2 / H2** (RISC-V, open NimBLE controller): requires a **binary patch
  of the prebuilt `$IDF_PATH/.../libble_app.a`** (change the min-interval floor from
  6→4 units). Provided as `tools/patch_nimble_5ms.py` (adapted from zhantss, MIT).
- **S3 / C3**: a different closed controller lib; historically a Kconfig
  (`CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE`, esp-idf#18467). Note: that symbol is
  **absent on IDF 6.0.1** — the S3 path needs re-verification on current IDF.

**Build integration (decision: opt-in, never silent).** The patch mutates the user's
global IDF install and is version-fragile (the RISC-V byte pattern is not guaranteed
across IDF versions). So it is gated behind a component Kconfig option
`SWITCH2_PRO_PATCH_NIMBLE_5MS` (default **n**). When enabled for a RISC-V target, the
component CMake invokes the patcher at configure time (idempotent, with `--verify-only`
first) and prints a loud notice. It is **not required for the GATT + pairing skeleton
milestone** — pairing runs over the command channel independent of the interval.

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
      4147423d-…       WRITE         firmware update (large)
      c765a961-…       NOTIFY        command response #1
      506d9f7d-…       NOTIFY        command response #2

Security: **no SMP** — the console app-level-pairs over the command channel and will
disconnect a peer that initiates SMP. We configure NimBLE not to initiate pairing;
the LTK from the 0x15 exchange is what encrypts the link. Bond (host addr + LTK)
persists in NVS for reconnect + wake.

## Milestones

1. **GATT + pairing skeleton (this milestone)**: custom GATT tree stands up,
   advertises with Nintendo manufacturer data, completes the 0x15 pairing handshake.
   Crypto host-verified; console-accepts-pairing is the on-hardware exit test.
2. Command dispatch + init sequence (flash/calibration reads, feature-select, LEDs,
   firmware-update-prompt suppression) so the console finishes bring-up.
3. Input report streaming (report 0x09: buttons incl. C/GL/GR, 12-bit sticks, IMU)
   at the console's cadence — needs the 5 ms patch for stability.
4. Wake-from-sleep advertisement (bonded reconnect with the 0x81 wake flag).

## Component layout

    switch2_pro/
      include/switch2_pro.hpp            Switch2Pro class (over BleGattServer)
      include/switch2_pro_protocol.hpp   UUIDs, command/subcommand ids, feature bits, fixed key, golden vector
      include/switch2_pro_report.hpp     Pro Controller 2 input report (0x09) packed struct
      src/switch2_pro.cpp                GATT setup, advertising, GAP, command dispatch
      src/switch2_pro_pairing.cpp        pairing crypto (mbedTLS) + state machine + self-test
      tools/patch_nimble_5ms.py          opt-in 5 ms connection-interval patcher (RISC-V)
      Kconfig                            SWITCH2_PRO_PATCH_NIMBLE_5MS opt-in
      example/                           C6-primary, S3-buildable
