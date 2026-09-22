# USB HID Host on the M5Stack Tab5 (SpaceMouse viewer)

Turns the Tab5 into a USB HID host and shows what the attached device sends,
on the screen: a 3Dconnexion **SpaceMouse** (SpaceNavigator, SpaceMouse
Compact / Wireless / Pro, ...) is decoded into its six axes and buttons, a
**keyboard** lights up its keys on a virtual keyboard, and any other HID device
(mouse, gamepad) shows its raw Input reports.

It is a bench tool for `espp::UsbHost` and for evaluating a SpaceMouse as a
6-DoF input, and a starting point for a Tab5 firmware that takes USB input.


## What it shows

- **Device card**: connection state (grey = none, green = SpaceMouse, blue =
  other HID), product / manufacturer, VID:PID, interface / protocol, report
  descriptor size.
- **Axes**: six centered bars, `Tx Ty Tz` (translation, blue) and `Rx Ry Rz`
  (rotation, orange), with the raw counts (`-350..350` on a SpaceNavigator).
- **Buttons**: one indicator per button (green while pressed).
- **Keyboard**: a boot-protocol keyboard (interface subclass 1, protocol 1)
  replaces the axes with a virtual US keyboard whose pressed keys light up.
  The example asks the keyboard for the boot protocol, a fixed 8-byte report
  (modifier bits, then up to six key usage ids), so no per-keyboard descriptor
  parsing is needed.
- **Last report**: the newest Input report's bytes and the report rate, plus
  per-report-id counts for a SpaceMouse.

`main/spacemouse_decoder.*` routes each raw report by its id byte to the
matching `espp::SpaceMouse*InputReport` class from `hid-rp` (1 = translation,
2 = rotation, 3 = buttons; the 12-byte combined report some newer firmware
sends on id 1 is handled too).

## Hardware

- M5Stack Tab5. The USB-A jack is on the ESP32-P4's high-speed USB-OTG
  controller (the USB Host Library's default on the P4) and the BSP enables its
  5 V through the IO expanders.
- The console (`idf.py monitor`) stays on the USB-C port: that is the other
  (full-speed) controller, with USB-Serial-JTAG, so both work at once.
- A wireless SpaceMouse works through its USB receiver.

## Build & flash

```sh
idf.py set-target esp32p4
idf.py -p <PORT> flash monitor
```

Built with the component manager on (the `usb` host library and `usb_host_hid`
class driver come from the registry, as in the plain `usb_host` example).

## Notes

- Axis signs follow the device: on a SpaceNavigator `Tx` + is right, `Ty` + is
  toward the user, `Tz` + is down (pushing the cap), and the rotations follow
  the right-hand rule about those axes. Check them against the bars before
  mapping to a robot frame.
- A SpaceMouse reports a zeroed translation + rotation pair when released, so
  the bars return to center on their own.
- A device that enumerates but is not opened as HID shows as "N USB device(s)
  enumerated, none opened as HID"; call `host.print_usb_devices()` (or set the
  `hid-host` log tag to debug with `CONFIG_LOG_MAXIMUM_LEVEL_DEBUG`) to see its
  interfaces.
- Devices with a VID other than 3Dconnexion's (`0x256F`) or the original
  Logitech-made SpaceNavigator (`0x046D`) are treated as generic HID.
