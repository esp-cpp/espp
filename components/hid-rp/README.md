# HID-RP Component

https://components.espressif.com/components/espp/hid-rp/badge.svg

The `hid-rp` component provides a wrapper around
https://github.com/intergatedcircuits/hid-rp and also provides an example
implementation of a configurable HID Gamepad using hid-rp.

It also implements Switch Pro and Xbox gamepad reports.

## Example

This example shows how to use the
[`hid-rp`](https://github.com/intergatedcircuits/hid-rp) library which is
bundled into the hid-rp component within espp.

It provides an example of a somewhat configurable HID Gamepad using the
`esppp::GamepadReport<>` template class.

