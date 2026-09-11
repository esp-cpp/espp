# Switch Pro (NS1) USB Example

This example emulates a **Nintendo Switch Pro controller** over the ESP32-S3's
native USB port, using the `espp::SwitchPro` protocol engine driven by the
`espp::usb_device` HID function.

It configures a single USB HID interface that advertises the Switch Pro report
descriptor (from `hid-rp`) with an interrupt-OUT endpoint, funnels received host
OUTPUT reports (the handshake) to `SwitchPro::on_hid_report()`, and streams input
reports once the host has enabled them — cycling the A / B / X / Y buttons and
sweeping the left stick so a connected Switch shows live input.

> **Emulation only.** The device advertises Nintendo's Pro Controller USB VID/PID
> (`0x057E` / `0x2009`) so a real Switch will bind it. Use it to test against a
> Switch you own.

## How to use

### Hardware Required

An ESP32-S3 (native USB-OTG). The console is on **UART0** (the native USB port is
taken by the HID interface, and on the ESP32-S3 the USB-Serial-JTAG console shares
that USB PHY) — connect a USB-UART adapter for `idf.py monitor`.

### Build and Flash

```sh
idf.py set-target esp32s3 build flash monitor
```

Then plug the ESP32-S3's **native USB** port into a Nintendo Switch (Home →
Controllers → Change Grip/Order) or a PC controller tester. The device enumerates
as "Pro Controller"; after the handshake you should see A/B/X/Y and the left stick
animating.

## Note

A full end-to-end handshake against real hardware requires a Switch (or a
Switch-Pro-aware host); this example is structurally complete and builds clean,
but the on-console handshake should be verified on hardware.
