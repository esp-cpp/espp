# espp BLDC Haptics USB Protocol

Framed binary protocol carried over the device's USB **vendor-specific**
interface (bInterfaceClass `0xFF`, one bulk IN + one bulk OUT endpoint). The
interface advertises WebUSB + MS OS 2.0 descriptors (via `espp::UsbDevice`), so
a Chromium browser can claim it without any driver — see `webapp/index.html`
for the reference host implementation.

- Default USB identity: VID `0x1209`, PID `0x0d34`, product string
  `espp BLDC Haptics`.
- Protocol version: `1` (reported in the INFO reply).

## Framing

Identical to the espp `ota` component's stream framing
(`components/ota/include/detail/ota_stream_protocol.hpp` is the authoritative
spec). All multi-byte fields are **little-endian**:

```
[magic u16 = 0x4F54 "OT"] [type u8] [len u32] [payload: len bytes] [crc32 u32]
```

- `magic`: u16 `0x4F54`; on the wire the bytes are `0x54 'T'` then `0x4F 'O'`.
- `type`: message type (tables below).
- `len`: payload length, capped at **4096** bytes per frame; receivers reject
  and resynchronize past any frame whose length field exceeds the cap.
- `crc32`: standard zlib CRC-32 (poly `0xEDB88320` reflected, init/final xor
  `0xFFFFFFFF`) over `magic..payload` (i.e. the 7 header bytes + payload).
  Golden check value: `crc32("123456789") == 0xCBF43926`.

Receivers parse incrementally and resynchronize on bad magic / oversized
length / CRC mismatch, so a corrupted stream recovers at the next intact frame.

### Flow control

**Commands are serialized**: the host sends one command frame and waits for its
reply (`OK` / `ERROR`, or the type-specific reply for the getters) before
sending the next. Two device-to-host frame kinds may arrive *unsolicited* and
must be tolerated at any time:

- `TELEMETRY (0x93)` — when streaming is enabled;
- `OTA_PROGRESS (0x83)` — informational during an OTA transfer.

The device suspends telemetry while an OTA session is active.

### Primitive types

- `u8`/`u16`/`u32`/`i32`: little-endian (two's complement for `i32`).
- `f32`: IEEE-754 single precision, little-endian.
- `str`: `u8` length followed by that many UTF-8 bytes (max 255).

## Host → device messages

| Type | Name          | Payload                                   | Reply |
|------|---------------|-------------------------------------------|-------|
| 0x01 | OTA_BEGIN     | `u32 image_size` (0 = unknown/streaming)  | OK(0) / ERROR |
| 0x02 | OTA_DATA      | raw image bytes (1..4096)                 | OK(total bytes written) / ERROR |
| 0x03 | OTA_FINISH    | —                                         | OK(total bytes written) / ERROR |
| 0x04 | OTA_ABORT     | —                                         | OK(bytes written) / ERROR |
| 0x10 | GET_INFO      | —                                         | INFO |
| 0x11 | GET_STATUS    | —                                         | STATUS |
| 0x12 | GET_MODES     | —                                         | MODES |
| 0x13 | SET_MODE      | `u8 mode_index`                           | OK(mode_index) / ERROR |
| 0x14 | SET_POSITION  | `i32 position` (detent index)             | OK(clamped position) / ERROR |
| 0x15 | SET_ENABLED   | `u8` 0 = disable, 1 = enable              | OK(0/1) / ERROR |
| 0x16 | PLAY_HAPTIC   | `f32 strength` (clamped to 0..10)         | OK(0) / ERROR |
| 0x17 | SET_STREAMING | `u8 enable` + `u16 period_ms` (5..1000; 0 = default 20) | OK(period_ms) / ERROR |
| 0x18 | GET_CRASH | none | CRASH |

Notes:

- **OTA** semantics are identical to the espp `ota` example: `OTA_BEGIN` erases
  the next OTA app partition (can take several seconds — use a generous
  timeout), `OTA_DATA` streams image bytes, `OTA_FINISH` validates the complete
  image (structure + appended SHA-256) and sets it as the boot partition, then
  the device **reboots ~750 ms after replying OK** (expect a USB disconnect).
  With bootloader rollback enabled the new app must mark itself valid on first
  boot or the bootloader rolls back. `OTA_DATA`/`OTA_FINISH`/`OTA_ABORT`
  without an active session yield `ERROR(operation_not_permitted)`.
- `SET_POSITION` re-labels the detent the knob is currently resting in: it sets
  the *logical* detent index (clamped to the active config's
  `[min_position, max_position]`) that position/telemetry values count from.
  The knob does **not** physically move — the motor keeps holding the current
  physical detent.
- `SET_ENABLED 0` de-energizes the motor driver via `BldcHaptics::stop()`
  (which calls `BldcMotor::disable()`, which calls `BldcDriver::disable()`),
  in addition to stopping the haptic control task; `1` re-enables it.
- `PLAY_HAPTIC` plays a short haptic "click" (a quick torque pulse in each
  direction). Rejected while disabled.

## Device → host messages

### OK (0x81)

`u32 value` — context-dependent (see the command table above).

### ERROR (0x82)

`u32 code` (a `std::errc` value) followed by a UTF-8 message.

### OTA_PROGRESS (0x83)

`u32 written` + `u32 total` (0 if unknown). Informational; may be ignored.

### INFO (0x90)

Reply to GET_INFO:

| Field            | Type | Description                        |
|------------------|------|------------------------------------|
| protocol_version | u8   | currently 1                        |
| project_name     | str  | firmware project name              |
| version          | str  | firmware version (git describe)    |
| build            | str  | compile date + time                |
| idf_version      | str  | ESP-IDF version                    |

### STATUS (0x91)

Reply to GET_STATUS:

| Field            | Type | Description                                      |
|------------------|------|--------------------------------------------------|
| mode_index       | u8   | active preset index                              |
| flags            | u8   | bit0 enabled, bit1 driver fault, bit2 streaming  |
| position         | i32  | current detent index                             |
| value            | f32  | continuous knob value (detent index + fraction)  |
| shaft_angle      | f32  | raw motor shaft angle, radians                   |
| shaft_velocity   | f32  | motor shaft velocity, radians/s                  |
| stream_period_ms | u16  | current telemetry period                         |

> Motor temperature / phase current are **not** reported: the supported boards
> (TMC6300 test stand, MotorGo Mini/Axis as driven by this example) expose no
> per-phase current or temperature telemetry to the firmware.

### MODES (0x92)

Reply to GET_MODES — enumerates the built-in `espp::detail` detent presets:

```
u8 count
repeated count times:
  u8  index            (the SET_MODE wire index)
  i32 min_position     (max < min means unbounded)
  i32 max_position
  f32 position_width   (radians between adjacent detents)
  f32 detent_strength
  f32 end_strength
  f32 snap_point
  u8  num_detent_positions
  i32 detent_positions[num]   (explicit "magnetic" detents; empty = all)
  str name
```

Current preset table (index → name):

| # | Preset |
|---|--------|
| 0 | Unbounded, no detents (`UNBOUNDED_NO_DETENTS`) |
| 1 | Bounded, no detents (`BOUNDED_NO_DETENTS`) |
| 2 | Multi-rev, no detents (`MULTI_REV_NO_DETENTS`) |
| 3 | On/off, strong detents (`ON_OFF_STRONG_DETENTS`) |
| 4 | Coarse values, strong detents (`COARSE_VALUES_STRONG_DETENTS`) — default |
| 5 | Fine values, no detents (`FINE_VALUES_NO_DETENTS`) |
| 6 | Fine values, with detents (`FINE_VALUES_WITH_DETENTS`) |
| 7 | Magnetic detents (`MAGNETIC_DETENTS`) |
| 8 | Return to center, with detents (`RETURN_TO_CENTER_WITH_DETENTS`) |

### TELEMETRY (0x93)

Sent periodically (every `stream_period_ms`, default 20 ms) while streaming is
enabled, the device is mounted, and no OTA session is active:

| Field          | Type | Description                                      |
|----------------|------|--------------------------------------------------|
| timestamp_ms   | u32  | device uptime, milliseconds (wraps)              |
| mode_index     | u8   | active preset index                              |
| flags          | u8   | bit0 enabled, bit1 driver fault, bit2 streaming  |
| position       | i32  | current detent index                             |
| value          | f32  | continuous knob value (detent index + fraction)  |
| shaft_angle    | f32  | raw motor shaft angle, radians                   |
| shaft_velocity | f32  | motor shaft velocity, radians/s                  |

The **continuous value** maps directly onto the knob geometry: the knob's
physical angle relative to the detent grid is `value * position_width` radians,
with `value` spanning `[min_position, max_position]` for bounded modes. This is
what the web app's dial renders.

### CRASH (0x94)

Reply to `GET_CRASH`. The payload is a UTF-8 text report of the previous
abnormal reset, or EMPTY when the boot history is clean. When the previous
reset was a panic with a flash core dump, the report includes the crashed
task, PC, and raw backtrace addresses (decode with
`xtensa-esp32s3-elf-addr2line -pfiaC -e build/bldc_haptics.elf <addrs>`);
brownout / watchdog resets are reported by reason (no core dump exists for
those). The web console requests this automatically after connecting and
prints the report in its log pane.
