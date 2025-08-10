# PI4IOE5V I2C GPIO Expander

[![Badge](https://components.espressif.com/components/espp/pi4ioe5v/badge.svg)](https://components.espressif.com/components/espp/pi4ioe5v)

The `Pi4ioe5v` component provides configuration and GPIO control for the
PI4IOE5Vxxxx family of I2C expanders. It derives from `espp::BasePeripheral`
and exposes simple helpers for direction, output, input, polarity inversion,
and pull configuration.

## Example

Use with `espp::I2c` via bound callbacks:

```cpp
//! [pi4ioe5v_example]
espp::I2c i2c({...});
espp::Pi4ioe5v exp({
  .device_address = espp::Pi4ioe5v::DEFAULT_ADDRESS,
  .direction_mask = 0x00, // all outputs on single 8-bit port
  .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                     std::placeholders::_2, std::placeholders::_3),
  .read_register = std::bind(&espp::I2c::read_at_register, &i2c,
                             std::placeholders::_1, std::placeholders::_2,
                             std::placeholders::_3, std::placeholders::_4),
  .write_then_read = std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1,
                               std::placeholders::_2, std::placeholders::_3,
                               std::placeholders::_4, std::placeholders::_5),
});
//! [pi4ioe5v_example]
```


