# Controller Component

[![Badge](https://components.espressif.com/components/espp/controller/badge.svg)](https://components.espressif.com/components/espp/controller)

The `Controller` class provides a convenience for reading multiple GPIOs at once
and mapping their state to common controller buttons. It can optionally be
configured to support joystick select, as well as to convert analog joystick
values into digital directional values (up/down/left/right). It can also be used
for just a subset of the buttons, should you wish to do so, by providing the
GPIO configuration for the unused buttons to be -1.

## Example

The [example](./example) shows how to use the `Controller` component to create
various different controller interface objects, such as:
* Digital input only controller (using the `DigitalConfig`) with only a couple of buttons (such as NES)
* Analog and digital controller (with 1 joystick that has x/y axes) using the `espp::OneshotAdc` class with the Joystick
* Analog controller with 2 joysticks using the `espp::Ads1x15` component to read 4 analog axes over I2C.

