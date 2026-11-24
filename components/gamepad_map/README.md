# Gamepad Map Component

[![Badge](https://components.espressif.com/components/espp/gamepad_map/badge.svg)](https://components.espressif.com/components/espp/gamepad_map)

The `gamepad_map` component remaps a gamepad's buttons and axes: button to
button, analog to digital, digital to analog and analog to analog, plus stick
dead zones. Maps are stored per controller type in a file with
[persistent_data](../persistent_data), together with how each controller type
plays a host's rumble (off, strength, strong / weak motors swapped). The
component is meant to sit between whatever reads a controller (a BLE or USB HID
gamepad, GPIO buttons) and whatever comes next (an emulated USB controller, an
emulator's input).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Gamepad Map Component](#gamepad-map-component)
  - [Overview](#overview)
  - [Gamepad state](#gamepad-state)
  - [Rules](#rules)
  - [Profiles](#profiles)
    - [Upgrading](#upgrading)
  - [From a HID gamepad](#from-a-hid-gamepad)
  - [Rumble](#rumble)
    - [Rumble options](#rumble-options)
  - [Web console](#web-console)
  - [Host test](#host-test)
  - [Examples](#examples)

<!-- markdown-toc end -->

## Overview

| Type | What it is |
|------|------------|
| `espp::GamepadState` | Buttons by position plus six axes: the common format inputs are decoded into |
| `espp::GamepadMapRule` | One remap: where one control's input goes |
| `espp::GamepadMap` | Stick dead zones plus a list of rules; `apply()` maps one state to another |
| `espp::GamepadProfiles` | A default map plus one map per controller type, each with its rumble options, saved in a file |
| `espp::RumbleOptions` | off / strength 0..200 % / swap strong-weak for a host's rumble; stored per profile; `apply()` once before encoding |
| `espp::to_gamepad_state()` | A state from a `hid_rp::GamepadReport` (`gamepad_state_hid_rp.hpp`) |
| `espp::GamepadMapService` | Serves a `GamepadProfiles` store to the web console (dispatcher module 11) |

Everything except `GamepadProfiles` is header-only plain data with no
allocation in `apply()`, so it can run on every report.

## Gamepad state

`espp::GamepadState` names face buttons by **position** (south / east / west /
north), not label, because the labels disagree between controller families:

| Position | Xbox | Nintendo | PlayStation |
|----------|------|----------|-------------|
| south    | A    | B        | Cross       |
| east     | B    | A        | Circle      |
| west     | X    | Y        | Square      |
| north    | Y    | X        | Triangle    |

Besides the face buttons there are L1/R1, L2/R2 (the triggers as buttons),
L3/R3, Select, Start, Home, Capture, the d-pad and four paddles. The axes are
the two sticks (-32767..32767, X right, Y **down**, as in HID) and the two
triggers (0..32767).

## Rules

| Kind | Effect | `param` |
|------|--------|---------|
| button → button | the source presses the target | - |
| axis → button | the target is pressed while the axis is past the threshold | threshold: positive = at or above, negative = at or below |
| button → axis | while pressed, the value is added to the target axis | value |
| axis → axis | the scaled source is added to the target axis | scale in percent, -400..400 (negative inverts) |

A control that is the source of a rule is **moved**: it no longer reaches its
own position unless the rule has `KEEP_SOURCE`. A rule with target `NONE`
removes its source. Several rules can feed one target: buttons are OR-ed and
axis contributions are summed, then clamped to the axis' range. Every control
that no rule takes as a source passes through unchanged, so an empty map is the
identity.

```cpp
espp::GamepadMap map;
map.swap(B::South, B::East)            // Nintendo-style A / B
    .left_stick_to_dpad()              // the stick also drives the d-pad
    .triggers_to_buttons()             // analog triggers also press ZL / ZR
    .add(espp::GamepadMapRule::button(B::Paddle1, B::South)); // a paddle as A
map.left_deadzone_percent = 8;

espp::GamepadState out = map.apply(in);
```

Check a map that came from outside (a file, a web page) with `valid()`.
`apply()` skips rules whose indices are out of range, so a bad map can never
read or write outside the state.

## Profiles

`espp::GamepadProfiles` keeps a default map and one `GamepadProfile` per
controller type, keyed by vendor and product ID. For BLE these come from the
Device Information service's PnP ID; for USB, from the device descriptor. A
controller that reports neither is matched by name. Every change is validated
and saved atomically. Look the map and the [rumble options](#rumble-options) up
when a controller connects, not on every report:

```cpp
espp::GamepadProfiles profiles({.file_path = fs.get_root_path() / "gamepad_profiles.bin",
                                .default_map = legacy_settings_as_a_map});
espp::GamepadMap active = profiles.map_for(vid, pid, name);
espp::RumbleOptions rumble = profiles.rumble_for(vid, pid, name);
```

The file is written on the first change. Until then the configured
`default_map` and `default_rumble` are used, which is how an application can
migrate its older settings into a map.

Every profile stores its controller type's rumble options next to its map, and
the default map has its own (`default_rumble()`) for every controller without a
profile. `rumble_for()` returns the profile's options, or the default's when
there is no profile. There are two ways to save a profile:

- `set_profile()` replaces the whole profile, rumble options included: a
  `{.vid, .pid, .name, .map}` without `.rumble` resets them to neutral.
- `set_profile_map()` changes only the map (and the stored name). An existing
  profile keeps its rumble options; a new one starts with the default's. The
  console saves a new profile the same way: its editor starts from the default
  map's rules and rumble options.

For the default, `set_default_map()` keeps the default rumble options,
`set_default_rumble()` changes only them, and `set_defaults()` saves both in
one write. `erase_profile()` removes the profile's rumble options with it (the
controller then uses the default's), and `reset()` sets the default rumble
options back to neutral too. Invalid options (see `RumbleOptions::valid()`) are
refused with `invalid_argument`, so nothing invalid is ever saved. Options read
from the file are checked as well: unknown flag bits are cleared and a strength
above 200 % is clamped to 200 %, each with a warning.

### Upgrading

Rumble options came with version 2 of the file's schema. A version 1 file
(written by firmware without rumble options) loads with neutral rumble options
everywhere, even when `Config::default_rumble` is set, and stays version 1 on
flash until the first change, which writes it as version 2. Older firmware
cannot read a version 2 file: it sets it aside as `<file>.unreadable` and starts
from its defaults. So rolling back to older firmware after a map or rumble
option was edited on the new one loses the profiles; rolling back before any
edit keeps them.

## From a HID gamepad

The runtime `hid_rp::GamepadDecoder` (`hid-rp-report-map.hpp`) decodes any HID
gamepad from its report descriptor. For a pass-through, turn off its emulator
conveniences (stick as d-pad, triggers as L2/R2, Capture as Home) so the map
decides those:

```cpp
auto report_map = espp::hid_rp::ReportMap::parse(descriptor);
if (!report_map || !espp::hid_rp::GamepadDecoder::looks_like_gamepad(*report_map))
  return; // not a gamepad (or a malformed descriptor)
espp::hid_rp::GamepadDecoder decoder(std::move(*report_map));
decoder.set_combine_controls(false);

espp::hid_rp::GamepadReport report;
if (decoder.update(report_id, payload, report))   // BLE: the ID comes out of band
  send(map.apply(espp::to_gamepad_state(report)));
```

## Rumble

`RumbleState` (`rumble_state.hpp`) is the output-direction counterpart of
`GamepadState`: how strongly to drive each motor (strong / weak, plus the two
impulse-trigger motors), 0..255. A host's rumble request is decoded into it, and
it is encoded into whatever drives the motors. `gamepad_state_hid_rp.hpp` bridges
it to `hid-rp`:

```cpp
// a Switch host's rumble (espp::SwitchPro::Config::on_rumble) to a BLE
// controller's rumble report (hid_rp::RumbleEncoder from its report map)
auto state = espp::to_rumble_state(switch_rumble);
auto payload = encoder.encode(espp::to_rumble_levels(state), 200);
```

### Rumble options

`espp::RumbleOptions` (`rumble_options.hpp`) says how one controller type plays
a host's rumble. It is two bytes of plain data, stored per profile:

| Field | Values | Effect |
|-------|--------|--------|
| `flags` bit0, `RUMBLE_OFF` | set / clear | off: every request plays as a stop (all motors 0) |
| `flags` bit1, `SWAP_MOTORS` | set / clear | the strong motor plays the weak motor's level and the weak motor the strong one's; the trigger motors are never swapped |
| `strength_percent` | 0..200, default 100 | every motor's level is scaled by this percentage |

`RumbleOptions{}` (on, 100 %, not swapped) is neutral: `apply()` returns the
request unchanged. `apply(request)` computes the levels to play:

- **Off** gives all four motors 0, whatever the strength or swap. The stored
  strength is kept, so turning rumble back on restores it.
- **Strength** sets each motor to `min(255, (level * strength_percent + 50) / 100)`
  (rounded half up). 0 % is silent, like off. Above 100 % each motor saturates
  at 255 on its own, so strong effects lose their differences: at 200 %, a
  level of 127 plays as 254 and anything from 128 up as 255. A level of 0 stays
  0 at every strength, so a stop is always a stop.
- **Swap** is for a controller whose motors come out the wrong way round. It
  commutes with the scaling. A controller with a single motor feels no
  difference, since `hid_rp::RumbleEncoder` drives it from the larger of strong
  and weak.
- Unknown flag bits are ignored, and a strength above 200 acts as 200.

The library never applies the options by itself. Apply them **once**, at send
time, to the host's raw request:

- Keep the host's latest request raw, and keep the options next to it, under
  the same lock.
- Each time the rumble sender decides what to send (a new request, an options
  change, a keep-alive repeat, a test pulse), compute `options.apply(raw)`.
  Deduplicate and repeat on that result.
- Send an all-zero result as a stop, including its repeats. Off means "send
  stops", not "send nothing", so turning rumble off mid-effect stops the motors.

```cpp
// each time the rumble sender sends: the host's raw request, shaped once
auto payload = encoder.encode(espp::to_rumble_levels(options.apply(host_request)), 200);
```

Never apply the options to an already-applied state: the scaling compounds and
a second swap undoes the first. Do not apply them where the host's request is
decoded either: a strength change would then wait for the host's next request,
and off would not stop a rumble the host is holding on. Look the options up
(`rumble_for()`, `GamepadMapService::active_rumble()`) at connect and in change
callbacks, not on every rumble update: the lookup takes the store's lock and may
allocate.

## Web console

`espp::GamepadMapService` serves a `GamepadProfiles` store over any framed byte
stream (WebUSB vendor, Web Serial CDC...) through an `espp::Dispatcher`. Its
protocol is `espp.gamepad_map` v1 on module 11 by default
(`include/detail/gamepad_map_protocol.hpp` is the wire spec). The hosted
[Gamepad Map Console](https://esp-cpp.github.io/espp/apps/gamepad_map_console.html)
uses it to edit the maps and their rumble options and to show the controller's
input before and after mapping, live. The application tells the service about
the controller and feeds it the input:

```cpp
espp::GamepadMapService service({.send = send_to_host,
                                 .profiles = &profiles,
                                 .on_active_map_changed = [&](const espp::GamepadMap &m) {
                                   std::lock_guard lock(map_mutex);
                                   active_map = m; // the console changed this controller's map
                                 },
                                 .on_active_rumble_changed = [&](const espp::RumbleOptions &r) {
                                   std::lock_guard lock(map_mutex);
                                   active_rumble = r; // ... or its rumble options: hand them
                                                      // to the rumble sender
                                 }});
dispatcher.register_module(service);

{
  std::lock_guard lock(map_mutex); // the callbacks above may run on another thread
  active_map = service.set_controller(vid, pid, name); // on connect; clear_controller() on disconnect
  active_rumble = service.active_rumble();             // right after, under the same lock
}
// for every report:
service.publish_input(raw, mapped); // a 32-byte copy under a lock; never sends
```

Each callback fires only when a change from the console gives the connected
controller a different value (or when the controller changed while the change
was being saved), and never while no controller is connected. The callbacks
run on the thread that handles the request; they must not call back into the
service or change the store.

The INPUT stream ends by itself when the host stops renewing it
(`Config::stream_lease`), so a closed browser tab does not leave it running.

Rumble options were added to the protocol without changing its version (still
`espp.gamepad_map` v1), so consoles and firmware from before them keep working
with newer ones. A console learns that a device has them from capability bits:

- INFO flags bit1: the device stores rumble options per map.
- MAP flags bit1: a 2-byte rumble block follows the map:
  `[flags u8][strength_percent u8]`, `RumbleOptions` field for field (neutral
  is `00 64`).
- SET_MAP takes the same block, optionally, after the map. With it, the map and
  the rumble options are saved together in one write. Without it (what a
  console from before rumble options sends), only the map is saved: the stored
  rumble options are kept, and a new profile gets the default's. Options with
  reserved flag bits or a strength above 200 are refused with an ERROR, and
  nothing is saved.

## Host test

`test/gamepad_map_host_test.cpp` covers the rule semantics, validation, the
hid-rp conversion (with a real BLE gamepad's descriptor), the rumble options'
math, and the profile store: its file round trip, the rumble options it keeps
per profile, and loading a version 1 file. `test/gamepad_map_protocol_host_test.cpp`
checks the wire protocol against golden encodings.
`test/gamepad_map_service_host_test.cpp` drives `GamepadMapService` with request
frames: replies, what each kind of SET_MAP saves, and which change callbacks
fire. Each file's build command is at its top, and CI runs them in
`.github/workflows/gamepad_map_tests.yml` together with the hid-rp report-map
test.

## Examples

- [example](./example) decodes a BLE gamepad report, remaps it, stores
  per-controller profiles on a LittleFS partition, and applies a profile's
  rumble options to a host's rumble request.
- [webapp_example](./webapp_example) serves the profiles over USB (WebUSB /
  Web Serial) to the Gamepad Map Console, with a demo controller.
