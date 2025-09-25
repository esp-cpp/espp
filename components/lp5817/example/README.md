# LP5817 Example

This example shows how to use the `espp::Lp5817` component to control an RGB LED over I2C.

It is designed for ESP32-S3 but can be adapted by changing the I2C pins.

## How to use example

### Hardware Required

This example can run on and ESP microcontroller with I2C connected to an LP5817
RGB LED driver. You should run `idf.py menuconfig` to set the I2C pins to match
your hardware configuration.

## Configuration

Run `idf.py menuconfig` to set the I2C pins to match your hardware
configuration, or select a predefined board configuration.

## Build and Flash

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output
