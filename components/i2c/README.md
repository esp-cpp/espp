# I2C Component

[![Badge](https://components.espressif.com/components/espp/i2c/badge.svg)](https://components.espressif.com/components/espp/i2c)

The `I2C` class provides a simple interface to the I2C bus. It is a wrapper
around the esp-idf I2C driver.

A helper `I2cMenu` is also provided which can be used to interactively test
I2C buses - scanning the bus, probing devices, reading and writing to devices.

Note that the `I2CMenu` is only available if you compile with `exception support
enabled` as it relies on the `Cli` component, which requires exceptions.

## Example

The [example](./example) shows how to use the `I2C` component to communicate with
peripherals on the I2C bus.
