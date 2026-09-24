# USB HID Host on the M5Stack Tab5 (HID device viewer)

Turns the Tab5 into a USB HID host and shows what the attached device sends,
on the screen. Every HID interface the host opens is decoded from its own
report descriptor: a **keyboard** lights up its keys on a virtual keyboard, a
**mouse** moves a dot around a pad (with its wheel and buttons), a **gamepad**
drives two stick pads, a d-pad and button indicators, and a 3Dconnexion
**SpaceMouse** (SpaceNavigator, SpaceMouse Compact / Wireless / Pro, ...) is
decoded into its six axes and buttons. Anything else shows its raw Input
reports.

It is a bench tool for `espp::UsbHost` and the `hid-rp` runtime report
decoders, for evaluating a SpaceMouse as a 6-DoF input, and a starting point
for a Tab5 firmware that takes USB input.


## What it shows

- **Device card**: connection state (grey = none, green = SpaceMouse, blue =
  keyboard / mouse / gamepad, orange = other HID), product / manufacturer,
  VID:PID, and one line per opened interface with what it was recognised as. A
  composite device (a keyboard with a mouse interface, or a wireless receiver
  carrying a keyboard, a mouse and a gamepad) lists all of them and shows a
  panel for each.
- **SpaceMouse**: six centered bars, `Tx Ty Tz` (translation, blue) and
  `Rx Ry Rz` (rotation, orange), with the raw counts (`-350..350` on a
  SpaceNavigator), and one indicator per button.
- **Keyboard**: a virtual US keyboard whose pressed keys and modifiers light
  up. The decoder reads the keyboard's own report layout, so boot-style
  keyboards (modifier byte + six key slots) and NKRO bitmaps both work, with no
  protocol switch.
- **Mouse**: a pad with a dot that follows the accumulated motion (clamped at
  the edges), a wheel bar with its accumulated count, and `L R M 4 5` button
  indicators.
- **Gamepad**: left and right stick pads with the raw stick values, the d-pad,
  the four face buttons by position (labelled with the Xbox letters for those
  positions: `A` south, `B` east, `X` west, `Y` north, whatever the pad calls
  them), and `L1 R1 L2 R2 L3 R3 Select Start Home`. Per-controller layout
  quirks are looked up by VID:PID in `hid-rp`.
- **Last report**: the newest Input report's bytes (from any interface) and
  the report rate.

The panel column scrolls by touch when more panels are attached than fit the
screen (a receiver with all three kinds, in landscape).

`main/spacemouse_decoder.*` routes each raw SpaceMouse report by its id byte
to the matching `espp::SpaceMouse*InputReport` class from `hid-rp` (1 =
translation, 2 = rotation, 3 = buttons; the 12-byte combined report some
newer firmware sends on id 1 is handled too). The keyboard, mouse and gamepad
decoders are `hid-rp`'s `espp::hid_rp::ReportMap` and the `KeyboardDecoder` /
`MouseDecoder` / `GamepadDecoder` built on it (`hid-rp-report-map.hpp`).

## Hardware

- M5Stack Tab5. The USB-A jack is on the ESP32-P4's high-speed USB-OTG
  controller (the USB Host Library's default on the P4) and the BSP switches
  its 5 V through an IO expander (`set_usb_a_power()`).
- The console (`idf.py monitor`) stays on the USB-C port: that is the other
  (full-speed) controller, with USB-Serial-JTAG, so both work at once.
- A wireless device works through its USB receiver, and a hub works too (hub
  support is on in the example's `sdkconfig.defaults`).
- Devices attached at power-up and hot-plugged devices both enumerate normally.
  The example keeps the jack's 5 V off until the host is listening and waits
  500 ms before powering the root port (both in menuconfig: **USB Host Tab5
  Example Configuration**).
- Seen during development, cause not established: with `idf.py monitor`
  attached to the USB-C port, a device on the jack sometimes stalled in
  enumeration, and never with the monitor detached. The firmware does not touch
  the USB-C controller, and other Tab5 firmware runs a console on USB-C with a
  device on USB-A continuously without stalls, so this is more likely the
  monitor's DTR/RTS handling (which resets the chip and can disturb the
  USB-Serial-JTAG PHY) than the hardware. If you hit it, try a terminal that
  leaves DTR/RTS alone, or a UART adapter.

## Build & flash

```sh
idf.py set-target esp32p4
idf.py -p <PORT> flash monitor
```

Built with the component manager on (the `usb` host library and `usb_host_hid`
class driver come from the registry, as in the plain `usb_host` example).

## Notes

- SpaceMouse axis signs follow the device: on a SpaceNavigator `Tx` + is
  right, `Ty` + is toward the user, `Tz` + is down (pushing the cap), and the
  rotations follow the right-hand rule about those axes. Check them against
  the bars before mapping to a robot frame. A SpaceMouse reports a zeroed
  translation + rotation pair when released, so the bars return to center on
  their own.
- Gamepad stick values are normalised to `-32767..32767` with Y growing
  downwards (the HID convention); a pad whose Y grows upwards is corrected by
  its VID:PID quirk.
- A device that enumerates but is not opened as HID shows as "N USB device(s)
  enumerated, none opened as HID"; call `host.print_usb_devices()` (or set the
  `hid-host` log tag to debug with `CONFIG_LOG_MAXIMUM_LEVEL_DEBUG`) to see its
  interfaces.
- A SpaceMouse is recognised by 3Dconnexion's vendor id (`0x256F`), or by the
  product ids of the early Logitech-branded SpaceNavigator / SpaceExplorer /
  SpacePilot (`0x046D:C62x`), before the descriptor is looked at: its six axes
  and buttons would otherwise pass for a gamepad.
