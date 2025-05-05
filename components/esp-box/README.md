# ESP-BOX Board Support Package (BSP) Component

The ESP32-S3-BOX and ESP32-S3-BOX-3 are development boards for the ESP32-S3
module. They feature a nice touchscreen display, an IMU, a speaker, microphones,
and expansion headers.

The `espp::EspBox` component provides a singleton hardware abstraction for
initializing the touch, display, and audio subsystems, as well as automatically
determining which version of the Box it's running on.

## Example

The [example](./example) shows how to use the `espp::EspBox` hardware
abstraction component to automatically detect and initialize components on both
the ESP32-S3-BOX and the ESP32-S3-BOX-3.
