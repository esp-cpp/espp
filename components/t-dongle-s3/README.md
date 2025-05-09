# LilyGo T-Dongle-S3 Board Support Package (BSP) Component

https://components.espressif.com/components/espp/t-dongle-s3/badge.svg

The LilyGo T-Dongle S3 is a development board for the ESP32-S3 module. It
features a USB-A connector which doubles as a micro-SD card reader, a color LCD,
an RGB LED, and a button.

The `espp::TDongleS3` component provides a singleton hardware abstraction for
initializing the display and LED subsystems.

## Example

The [example](./example) shows how to use the `espp::TDongleS3` hardware
abstraction component initialize the components on the LilyGo T-Dongle-S3.

