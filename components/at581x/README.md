# AT581X Radar Presence Sensor Component

[![Badge](https://components.espressif.com/components/espp/at581x/badge.svg)](https://components.espressif.com/components/espp/at581x)

The `At581x` component provides a driver for the **AT581X** (AirTouch) 5.8 GHz
microwave-radar human-presence / motion sensor. This is the radar found on
modules such as the MoreSense **MS58-3909S68U4**, including the one on the
**ESP32-S3-BOX-3 sensor / dock board** (`esp32-s3-box-3-sensor-01`), where it is
wired to I2C `SDA=GPIO41`, `SCL=GPIO40`, and its presence/motion output to
`GPIO21`.

## How it works

The AT581X is **configured over I2C** (detection distance / sensitivity, RF
frequency, gain, power consumption, and timing). Detection itself is reported on
a dedicated **active-high output GPIO** that stays asserted for the configured
`trigger_keep_time` after the last detection. So:

- Use `At581x` (this component) to configure the radar over I2C.
- Use a GPIO interrupt (e.g. `espp::Interrupt`) on the radar's output pin to
  react to presence/motion.

## Example

The [example](./example) configures the radar over I2C, demonstrates changing
the sensitivity / RF state at runtime, and (if the radar's output GPIO is
configured) prints presence transitions using `espp::Interrupt`.

```cpp
#include "at581x.hpp"
#include "i2c.hpp"

espp::I2c i2c({.port = I2C_NUM_0, .sda_io_num = GPIO_NUM_41, .scl_io_num = GPIO_NUM_40});
auto dev = i2c.add_device<uint8_t>({.device_address = espp::At581x::DEFAULT_ADDRESS}, ec);

espp::At581x radar({
    .write = espp::make_i2c_addressed_write(dev),
    .read_register = espp::make_i2c_addressed_read_register(dev),
    .sensing_distance = 700, // 0..1023, larger = farther / more sensitive
    .log_level = espp::Logger::Verbosity::INFO,
});

// react to presence on the radar's output GPIO:
espp::Interrupt presence({.interrupts = {{
    .gpio_num = RADAR_OUT_GPIO,
    .callback = [](const auto &e) { fmt::print("presence: {}\n", e.active); },
    .active_level = espp::Interrupt::ActiveLevel::HIGH,
    .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
}}});
```

## Configuration notes

- `sensing_distance` (0..1023): larger values increase the detection range /
  sensitivity (it is internally converted to the chip's detection-threshold
  delta of `1023 - sensing_distance`).
- `frequency_mhz` must be one of `At581x::allowed_frequencies_mhz()` and
  `power_consumption_ua` one of `At581x::allowed_power_ua()`.
- After changing any setting, the driver re-writes the configuration and resets
  the RF frontend so it takes effect.
