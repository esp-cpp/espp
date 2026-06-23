# XIAO ESP32S3 Sense Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/xiao-esp32s3-sense/badge.svg)](https://components.espressif.com/components/espp/xiao-esp32s3-sense)

The Seeed Studio XIAO ESP32S3 Sense combines the XIAO ESP32S3 module with the
Sense expansion board, which adds an OV2640 camera, a PDM microphone, and a
microSD card slot.

The `espp::XiaoEsp32S3Sense` component provides a singleton hardware abstraction
for the board's documented camera, microphone, user-LED, and microSD pin
mappings.

It also provides helper methods for creating default ESP-IDF PDM RX I2S
configuration structures for the onboard microphone, TimerCam-style user LED
control helpers including Gaussian breathing, and a T-Deck / T-Dongle-S3 style
microSD mount helper.

Published helpers include:

- `camera_pins()`
- `microphone_pins()`
- `sd_card_pins()`
- `initialize_sdcard()`
- `sdcard()`
- `user_led_pin()`
- `initialize_led()`
- `start_led_breathing()`
- `set_led_breathing_period()`
- `set_led_brightness()`
- `gaussian()`

## Example

The [example](./example) shows how to use the `espp::XiaoEsp32S3Sense`
component in a streamer-style application that serves OV2640 MJPEG video and
PDM-microphone PCM audio over RTSP while driving the onboard user LED with the
same breathing behavior used by the ESP32 Timer-Cam example, and performing a
simple microSD mount/write/read smoke test when a card is present.
