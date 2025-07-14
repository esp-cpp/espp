# Waveshare ESP32-S3 TouchLCD Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/ws-s3-touch/badge.svg)](https://components.espressif.com/components/espp/ws-s3-touch)

The Waveshare ESP32-S3 TouchLCD is a development board for the ESP32-S3 module.
It features a nice touchscreen display, an IMU, a buzzer, and a real-time clock
(RTC).

The `espp::WsS3Touch` component provides a singleton hardware abstraction for
initializing the touch, display, IMU, and audio subsystems.

## Example

The [example](./example) shows how to use the `espp::WsS3Touch` hardware
abstraction component to automatically detect and initialize components on the
Waveshare ESP32-S3 TouchLCD board.

