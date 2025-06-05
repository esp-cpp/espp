# ALXV Labs Byte90 Board Support Package (BSP) Component 

[![Badge](https://components.espressif.com/components/espp/byte90/badge.svg)](https://components.espressif.com/components/espp/byte90)

The ALXV Labs Byte90 is a cute little retro computer styled ESP32-S3 system. It
features a nice OLED display, integrated battery + charging circuit, ADXL345
3-axis I2C accelerometer, and button.

The `espp::Byte90` component provides a singleton hardware abstraction for
initializing the accelerometer, interrupts, and display subsystems.

## Example

The [example](./example) shows how to use the `espp::Byte90` hardware abstraction
component initialize the components on the ALXV Labs Byte90.

