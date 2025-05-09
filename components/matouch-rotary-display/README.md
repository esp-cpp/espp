# Matouch-Rotary-Display Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/matouch-rotary-display/badge.svg)](https://components.espressif.com/components/espp/matouch-rotary-display)

The Matouch Rotary Display is a development board for the ESP32-S3 module. It
features a nice touchscreen display, a rotary encoder, a push button behind the
screen, and some expansion headers along with other peripherals like a micro-SD
card.

The `espp::MatouchRotaryDisplay` component provides a singleton hardware
abstraction for initializing the touch, display, and button.

## Example

The [example](./example) shows how to use the `espp::MatouchRotaryDisplay`
hardware abstraction component initialize the components on the [MaTouch Rotary
Display](https://wiki.makerfabs.com/MaTouch_ESP32_S3_Rotary_IPS_Display_1.28_GC9A01.html).

