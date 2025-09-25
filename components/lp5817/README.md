# LP5817 - RGB LED Driver (I2C)

[![Badge](https://components.espressif.com/components/espp/lp5817/badge.svg)](https://components.espressif.com/components/espp/lp5817)

This component provides a C++ driver for the TI LP5817 RGB LED driver over I2C,
implemented using `espp::BasePeripheral`.

- 3 LED channels (R, G, B) with 8-bit brightness, controllable via analog
  dot-current control or 26 KHz PWM control with linear or exponential dimming
- Programmable fade (ramp) times and step sizes
- Global enable/shutdown
- Open short detection/status

See the [example](./example) for usage.
