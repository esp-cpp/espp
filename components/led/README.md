# LED Component

[![Badge](https://components.espressif.com/components/espp/led/badge.svg)](https://components.espressif.com/components/espp/led)

The LED provides a convenient and thread-safe wrapper around the [ESP-IDF LEDC
perhipheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html#led-control-ledc).

It allows for both instant and hardware-based timed changing (fading) of duty
cycle (in floating point `percent [0,100]`).

## Example

The [example](./example) shows how to use the `espp::Led` class to control the
LED using the ESP32 hardware.

