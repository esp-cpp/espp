# AW9523 I2C I/O Expander and LED Driver

[![Badge](https://components.espressif.com/components/espp/aw9523/badge.svg)](https://components.espressif.com/components/espp/aw9523)

The `AW9523` I/O expander component allows the user to configure inputs,
outputs, interrupts, etc. via a serial interface such as I2C. It also supports
dimming control for LEDs attached to the expander's port pins, when those pins
are configured into a special LED mode.

## Example

The [example](./example) shows how to use the `Aw9523` component to communicate
(via I2C) with an AW9523 I2C digital IO expander and LED driver.

