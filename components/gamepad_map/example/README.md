# Gamepad Map Example

This example shows the `gamepad_map` component between a gamepad and whatever
comes next:

1. A BLE gamepad's HID report map (a ShanWan BSP-D8 in its Android mode) is
   parsed with `hid_rp::ReportMap` and decoded with `hid_rp::GamepadDecoder`,
   with the decoder's emulator conveniences turned off.
2. Each decoded report becomes an `espp::GamepadState`, which an
   `espp::GamepadMap` remaps. The map here swaps A/B to the Nintendo layout,
   puts the left stick on the d-pad as well, makes the analog triggers press
   ZL/ZR too, and adds a left-stick dead zone.
3. `espp::GamepadProfiles` stores a default map and a per-controller-type
   profile in a file on a [littlefs](https://github.com/littlefs-project/littlefs)
   partition, and reloads them.

It ends with a pass / fail summary.

## How to use example

### Hardware Required

Any ESP32 with a `littlefs` partition (see `partitions.csv`). The partition
label is configured through `menuconfig`
(`CONFIG_ESPP_FILE_SYSTEM_PARTITION_LABEL`).

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Expected Output

```
I (…) gamepad map example: Starting gamepad map example!
I (…) gamepad map example: Decode a BLE gamepad report, then remap it
I (…) gamepad map example: controller: {buttons: [south], lx: -32767, ly: 257, rx: 257, ry: 257, lt: 0, rt: 32767}
I (…) gamepad map example: mapped:     {buttons: [east, r2, left], lx: -32767, ly: 0, rx: 257, ry: 257, lt: 0, rt: 32767}
  [PASS] south is mapped to east
  [PASS] the left stick also presses d-pad left
  [PASS] the stick is kept
  [PASS] the right trigger also presses R2
I (…) gamepad map example: Per-controller profiles in a file
I (…) gamepad map example: BSP-D8 map: 4 rule(s); default map: 2 rule(s)
  [PASS] the profile is used for its controller
  [PASS] other controllers get the default map
  [PASS] profiles survive a reload
  [PASS] erase a profile
I (…) gamepad map example: 8 passed, 0 failed
I (…) gamepad map example: Gamepad map example complete!
```
