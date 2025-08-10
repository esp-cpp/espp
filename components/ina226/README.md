# INA226 I2C Current/Power Monitor

[![Badge](https://components.espressif.com/components/espp/ina226/badge.svg)](https://components.espressif.com/components/espp/ina226)

The `Ina226` component provides simple APIs to configure and read voltage,
current, and power from a TI INA226 over I2C. It derives from
`espp::BasePeripheral` and uses 8-bit register addressing.

## Example

The [example](./example) shows how to use the `Ina226` component to read bus
voltage, current, and power via I2C.


