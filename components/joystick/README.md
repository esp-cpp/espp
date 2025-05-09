# Joystick Component

[![Badge](https://components.espressif.com/components/espp/joystick/badge.svg)](https://components.espressif.com/components/espp/joystick)

The `Joystick` class provides a wrapper around a 2-axis analog joystick, with an
associated reader function for grabbing the raw values. When the joystick
`update()` is called, the raw values are mapped into the range `[-1,1]` for each
axis according to the configuration provided.

## Example

The [example](./example) shows the use of the `Joystick` class to manage the
input from and perform mapping / calibration of joystick data from an analog
joystick.

