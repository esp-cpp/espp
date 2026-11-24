# HID-RP Component

[![Badge](https://components.espressif.com/components/espp/hid-rp/badge.svg)](https://components.espressif.com/components/espp/hid-rp)

The `hid-rp` component provides a wrapper around
https://github.com/intergatedcircuits/hid-rp and also provides an example
implementation of a configurable HID Gamepad using hid-rp.

It also implements Switch Pro, Xbox gamepad, and 3Dconnexion SpaceMouse reports.

## Runtime report maps

`hid-rp-report-map.hpp` goes the other way: given the report descriptor of a
device found at runtime (a USB HID device, a BLE HID service), `ReportMap::parse()`
works out where every field sits. Input fields (`fields()`) feed the decoders
(`GamepadDecoder`, `KeyboardDecoder`, ...). Output fields (`output_fields()`,
host to device: rumble, LEDs) are mapped too, each with its unit.

## Rumble

`hid-rp-rumble.hpp` handles rumble in both directions of a controller bridge:

- **To a controller:** `RumbleEncoder::from_report_map()` finds a Physical
  Interface Device style output report in the controller's own report map (DC
  Enable Actuators, one Magnitude per actuator, Duration, Start Delay, Loop
  Count, as the Xbox Wireless controller declares over Bluetooth LE). Any
  controller that declares one can be driven, whatever its VID/PID:

  ```cpp
  auto map = espp::hid_rp::ReportMap::parse(report_descriptor);
  auto rumble = map ? espp::hid_rp::RumbleEncoder::from_report_map(*map) : std::nullopt;
  if (rumble) {
    // strong (left) motor full, weak (right) motor half, for 200 ms
    auto payload = rumble->encode({.strong = 255, .weak = 128}, 200);
    write_output_report(rumble->report_id(), payload); // e.g. a BLE HID output report
  }
  ```

  The descriptor gives each field's position and range. Which Magnitude drives
  which motor follows the Xbox convention (left trigger, right trigger, strong,
  weak with four actuators; strong, weak with two), and `set_motor()` overrides it.
  An effect plays once, for its duration: re-send it while the rumble lasts, so a
  lost stop report cannot leave the motors running.
- **From a Switch host:** `decode_switch_rumble_report()` reads the two motor
  levels (HD rumble, both bands) from the output reports `0x01` and `0x10` a
  Switch sends a Pro Controller. `espp::SwitchPro` delivers them through
  `Config::on_rumble`.

## Example

This example shows how to use the
[`hid-rp`](https://github.com/intergatedcircuits/hid-rp) library which is
bundled into the hid-rp component within espp.

It provides an example of a somewhat configurable HID Gamepad using the
`esppp::GamepadReport<>` template class.

