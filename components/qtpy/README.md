# QtPy Board Support Package (BSP) Component 

[![Badge](https://components.espressif.com/components/espp/qtpy/badge.svg)](https://components.espressif.com/components/espp/qtpy)

The QtPy ESP32 Pico and QtPy ESP32-S3 are development boards for the ESP32 Pico
and ESP32-S3, respectively. They feature a USB-C connector, a QWIIC I2C
connector, 0.1" headers, an RGB LED, and a button.

The `espp::QtPy` component provides a singleton hardware abstraction for
initializing the Button, I2C, and LED subsystems.

## Example

This example shows the use of the `QtPy` class to perform hardware
initialization for the QtPy ESP32 Pico and the QtPy ESP32-S3.
