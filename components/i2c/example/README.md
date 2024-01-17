# I2C Example

This example shows how to use the `I2C` component to communicate with
peripherals on the I2C bus.

It supports using `menuconfig` to configure
* i2c pins (sda /scl), with configuration pre-selected for QtPy ESP32 PICO and QtPy ESP32s3
* i2c device address
* i2c device register address
* Number of bytes to read from the i2c device register (register size)

## How to use example

Configure the example via `menuconfig`:

![CleanShot 2024-01-17 at 13 36 51](https://github.com/esp-cpp/espp/assets/213467/3ade0226-cf09-47cf-b601-22569a6da346)

### Hardware Required

This example requires a connection (via I2C) to an I2C device.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2024-01-17 at 13 40 38](https://github.com/esp-cpp/espp/assets/213467/3865e661-eee4-4917-8460-25e7a0b87ae7)
