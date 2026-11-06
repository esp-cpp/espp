# X-Input (Xbox 360) controller example

Presents the ESP32-S3 as a wired **Xbox 360 controller** over the native USB-OTG
peripheral, using `espp::UsbDevice`'s X-Input function. A PC's XUSB driver binds
it, so it shows up as an Xbox 360 controller in Windows' "Set up USB game
controllers" (`joy.cpl`) and any X-Input game, or under Linux `xpad`.

The demo sweeps the sticks/triggers in a circle and steps the face buttons
A/B/X/Y one at a time each second, so you can see live input, and logs any
rumble / LED reports the host sends back. The console/logs go to **UART0** (with
USB-Serial-JTAG as an early-boot secondary): on the ESP32-S3 the USB-Serial-JTAG
controller shares the native USB port's PHY with USB-OTG, so keeping the console
on it would contend with the emulated controller interface and reboot-loop the
device. Use a UART / USB-UART adapter on UART0 for `idf.py monitor`.

## Build & flash

X-Input is served by a custom TinyUSB application class driver built into the
`usb_device` component, so it needs **no** built-in USB class enabled — the
example's `sdkconfig.defaults` disables CDC/vendor/HID entirely. Flash it to an
ESP32-S3 and plug the native USB-OTG port into a PC:

```sh
idf.py -p <PORT> flash monitor   # console is on UART0 (USB-UART adapter)
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
[README](../README.md#enabling-x-input-xbox-360) for how a PC recognizes the
controller in each configuration.

## Composite variant: a WebUSB console beside the controller

A device that is also something else, a controller with a browser-based
configuration console on a WebUSB vendor interface for instance, cannot present
the retail controller's identity: that identity (and a 0xFF device class) makes
Windows' Xbox driver take the whole device, hiding the other interface. The
`sdkconfig.composite` variant builds that case:

```sh
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.composite" -p <PORT> flash monitor
```

It enables TinyUSB's built-in vendor class driver (the X-Input driver only
claims its own interface, so the two coexist) and `XINPUT_EXAMPLE_COMPOSITE`,
which adds a WebUSB vendor interface and gives the device Microsoft's vendor ID
with a product ID of its own (`XINPUT_EXAMPLE_COMPOSITE_PID`, 0x0E60 by default).
The device keeps the composite identity (0x00 device class), and the Microsoft
OS 2.0 descriptor that gives the vendor interface WinUSB also carries the `XUSB10`
compatible ID for the X-Input interface, which Windows' Xbox 360 driver matches
(`USB\MS_COMP_XUSB10`). Linux's xpad matches the interface class under the
Microsoft vendor ID.

The product ID must be one Windows' Xbox driver INF does **not** list by hardware
ID (the retail controllers' 0x028E, 0x0719, ... are), or that hardware-ID match
again takes the whole device. Still Microsoft's vendor ID: emulation / testing
of your own device only.
