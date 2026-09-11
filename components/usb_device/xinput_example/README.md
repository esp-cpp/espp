# X-Input (Xbox 360) controller example

Presents the ESP32-S3 as a wired **Xbox 360 controller** over the native USB-OTG
peripheral, using `espp::UsbDevice`'s X-Input function. A PC's XUSB driver binds
it, so it shows up as an Xbox 360 controller in Windows' "Set up USB game
controllers" (`joy.cpl`) and any X-Input game, or under Linux `xpad`.

The demo sweeps the sticks/triggers in a circle and steps the face buttons
A/B/X/Y one at a time each second, so you can see live input, and logs any
rumble / LED reports the host sends back. The console/logs go to the separate
built-in USB-Serial-JTAG so they stay off the emulated controller interface.

## Build & flash

X-Input is served by a custom TinyUSB application class driver built into the
`usb_device` component, so it needs **no** built-in USB class enabled — the
example's `sdkconfig.defaults` disables CDC/vendor/HID entirely. Flash it to an
ESP32-S3 and plug the native USB-OTG port into a PC:

```sh
idf.py -p <PORT> flash monitor   # monitor is the USB-Serial-JTAG console
```

## Identity (emulation only)

The device enumerates with **Microsoft's Xbox 360 VID/PID (`0x045E:0x028E`)** and
device class `0xFF/0xFF/0xFF` — that identity is what makes the host's XUSB driver
bind it. These are Microsoft's identifiers, for **emulation / testing of your own
device only**; a shipped product must not enumerate under them.

## Using it in your own code

```cpp
espp::UsbDevice::Config cfg;
espp::UsbDevice::XInputFunction xinput;      // default VID/PID = Xbox 360 wired
xinput.on_rumble = [](std::span<const uint8_t> data) { /* drive motors / LEDs */ };
cfg.xinput = xinput;                          // X-Input as the ONLY function
espp::UsbDevice usb(cfg);
std::error_code ec;
usb.initialize(ec);

espp::xinput::GamepadState state;
state.set(espp::xinput::Button::A, true);
state.lx = 20000;                             // left stick X
usb.update_xinput_state(state);                    // send a 20-byte input report
```

See `components/usb_device/include/xinput.hpp` for the `GamepadState` /
`Button` API and the report layout, and the component
[README](../README.md#enabling-x-input-xbox-360) for the constraints (X-Input must
be the only enabled function).
