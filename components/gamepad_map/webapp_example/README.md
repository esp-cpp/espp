# Gamepad Map Web App Example

This example serves an `espp::GamepadProfiles` store over USB with
`espp::GamepadMapService`, for the hosted
[Gamepad Map Console](https://esp-cpp.github.io/espp/apps/gamepad_map_console.html).
The console connects on the vendor (WebUSB) or CDC (Web Serial) interface. It
edits the default map and the per-controller-type profiles (button / axis
rules, stick dead zones, presets, binding a control by pressing it, and the
rumble options: off, strength, swap the strong / weak motors) and shows the
controller's input before and after mapping, live.

There is no real controller. A demo controller task generates input at 100 Hz:

- the left stick circles every 4 s
- the right stick sweeps left and right
- the triggers ramp
- all within 40% of full travel, below the console's 50% "learn" threshold
- the BOOT button (`CONFIG_EXAMPLE_BUTTON_GPIO`: GPIO 0 on the ESP32-S3, GPIO
  35 on the ESP32-P4 dev kits) is its South button, so "learn" picks the
  button you press

A real application decodes its controller instead (`hid_rp::GamepadDecoder` +
`espp::to_gamepad_state()`) and sends the mapped state on to a USB controller,
an emulator, and so on. The demo controller has no rumble motors either: the
example keeps the connected controller's rumble options (from
`GamepadMapService::active_rumble()` at connect and
`Config::on_active_rumble_changed` after a change) and logs each change. A real
application hands them to its rumble sender, which applies them
(`espp::RumbleOptions::apply()`) once to each host rumble request it sends.
It also answers the console's rumble **Test** (`Config::on_test_rumble`) by
logging the options it would play the test pattern with.

The device also serves the standard espp USB services: system info / reboot
(module 7), the heap / task monitor (8), OTA (0) and core dump (4). The
profiles live in a file on the 128 KB `littlefs` partition (see
`partitions.csv`).

## How to use example

### Hardware Required

An ESP32-S3 (or ESP32-P4) board with its native USB connector (USB-OTG). For
logs, connect a USB-UART adapter to UART0; the native USB port carries the
vendor and CDC interfaces.

### Build and Flash

The example builds against the components from the registry by default. In this
repository (as CI does), build it with the component manager off:

```
IDF_COMPONENT_MANAGER=0 idf.py build
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Use the console

Open the [Gamepad Map Console](https://esp-cpp.github.io/espp/apps/gamepad_map_console.html)
(WebUSB / Web Serial need Chrome or Edge), click Connect and pick the device.
The demo controller shows up as `1209:0001 "Demo controller"`. Turn the live
view on, edit its profile or the default map, and press Save. The mapped side of
the live view changes as soon as the device has the new map.

To try the rumble options, open the demo controller's profile (or the default
map, which the demo controller uses while it has no profile) and change the
Rumble row under the dead zones: untick "on", move the strength slider, or tick
"swap strong / weak motors". Save sends them with the map. Nothing can be felt,
since the demo controller has no motors, but the device logs the new options
(see below), and loading the map again shows them. With this firmware the status
line under the Connect buttons shows "Rumble options: per map".

**Test** (next to the swap box) sends the options as they are in the editor,
before Save: the device logs "Rumble test requested: off=... strength=...%
swap=..." and the console says the test was played. A real application plays a
short pattern on the controller instead.

## Example Output

```
I (…) Gamepad Map Console: Starting USB<->gamepad map console example
I (…) Gamepad Map Console: Clean boot history (reset reason: power-on)
I (…) Gamepad Map Console: Gamepad map console ready. Connect the web app over WebUSB / Web Serial.
I (…) Gamepad Map Console: Map changed from the console: 2 rule(s)
I (…) Gamepad Map Console: Rumble options changed from the console: off=false strength=150% swap=true
```
