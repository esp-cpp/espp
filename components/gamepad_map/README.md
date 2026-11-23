# Gamepad Map Component

[![Badge](https://components.espressif.com/components/espp/gamepad_map/badge.svg)](https://components.espressif.com/components/espp/gamepad_map)

The `gamepad_map` component remaps a gamepad's buttons and axes: button to
button, analog to digital, digital to analog and analog to analog, plus stick
dead zones. Maps are stored per controller type in a file with
[persistent_data](../persistent_data). The component is meant to sit between
whatever reads a controller (a BLE or USB HID gamepad, GPIO buttons) and
whatever comes next (an emulated USB controller, an emulator's input).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Gamepad Map Component](#gamepad-map-component)
  - [Overview](#overview)
  - [Gamepad state](#gamepad-state)
  - [Rules](#rules)
  - [Profiles](#profiles)
  - [From a HID gamepad](#from-a-hid-gamepad)
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
| `espp::GamepadProfiles` | A default map plus one map per controller type, saved in a file |
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
and saved atomically. Look the map up when a controller connects, not on every
report:

```cpp
espp::GamepadProfiles profiles({.file_path = fs.get_root_path() / "gamepad_profiles.bin",
                                .default_map = legacy_settings_as_a_map});
espp::GamepadMap active = profiles.map_for(vid, pid, name);
```

The file is written on the first change. Until then the configured
`default_map` is used, which is how an application can migrate its older
settings into a map.

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

## Web console

`espp::GamepadMapService` serves a `GamepadProfiles` store over any framed byte
stream (WebUSB vendor, Web Serial CDC...) through an `espp::Dispatcher`. Its
protocol is `espp.gamepad_map` v1 on module 11 by default
(`include/detail/gamepad_map_protocol.hpp` is the wire spec). The hosted
[Gamepad Map Console](https://esp-cpp.github.io/espp/apps/gamepad_map_console.html)
uses it to edit the maps and to show the controller's input before and after
mapping, live. The application tells the service about the controller and feeds
it the input:

```cpp
espp::GamepadMapService service({.send = send_to_host,
                                 .profiles = &profiles,
                                 .on_active_map_changed = [&](const espp::GamepadMap &m) {
                                   std::lock_guard lock(map_mutex);
                                   active_map = m; // the console changed this controller's map
                                 }});
dispatcher.register_module(service);

{
  std::lock_guard lock(map_mutex); // the callback above may run on another thread
  active_map = service.set_controller(vid, pid, name); // on connect; clear_controller() on disconnect
}
// for every report:
service.publish_input(raw, mapped); // a 32-byte copy under a lock; never sends
```

The INPUT stream ends by itself when the host stops renewing it
(`Config::stream_lease`), so a closed browser tab does not leave it running.

## Host test

`test/gamepad_map_host_test.cpp` covers the rule semantics, validation, the
hid-rp conversion (with a real BLE gamepad's descriptor) and the profile store's
file round trip. `test/gamepad_map_protocol_host_test.cpp` checks the wire
protocol against golden encodings. The build command is at the top of the file, and CI runs it in
`.github/workflows/gamepad_map_tests.yml` together with the hid-rp report-map
test.

## Examples

- [example](./example) decodes a BLE gamepad report, remaps it, and stores
  per-controller profiles on a LittleFS partition.
- [webapp_example](./webapp_example) serves the profiles over USB (WebUSB /
  Web Serial) to the Gamepad Map Console, with a demo controller.
