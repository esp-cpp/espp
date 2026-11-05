# Smart Panlee SC01 Plus Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/smartpanlee-sc01-plus/badge.svg)](https://components.espressif.com/components/espp/smartpanlee-sc01-plus)

The Smart Panlee SC01 Plus (ZX3D50CE08S-USRC / WT32-SC01-PLUS class hardware) is
an ESP32-S3 touchscreen display module with a 3.5-inch 320x480 ST7796 LCD, an
FT5x06-family capacitive touch controller, onboard speaker wiring via I2S, SPI
microSD support, and RS-485 plus expansion GPIOs.

The `espp::SmartPanleeSc01Plus` component provides a singleton hardware
abstraction for the board's display, touch controller, backlight, speaker audio
path, microSD card, and exposed peripheral pin mappings.

## Supported Features

- 3.5-inch 320x480 ST7796 display over 8080 8-bit parallel bus
- FT5x06 capacitive touch over I2C
- PWM backlight brightness control
- I2S speaker playback with software volume, mute, and sample-rate control
  (`initialize_audio()`, `play_audio()`, `volume()`, through the `audio`
  component)
- SPI microSD card mounting helpers
- Exposed I2S, RS-485, and external GPIO pin maps for application use
- Object-style `espp::St7796` display-controller integration for the LCD path

## Notes

- This first BSP release focuses on the board features that are directly wired
  and well documented in the published board references.
- The board's speaker path is driven over I2S by `espp::Audio` (the `audio`
  component) on the published `BCLK`, `LRCK`, and `DOUT` pins, without a
  codec: volume and mute are applied in software. `initialize_audio()` takes
  an `AudioConfig` (sample rate, playback buffer size / heap caps, task
  configuration); `audio()` returns the `espp::Audio` object. The older
  `initialize_audio(sample_rate[, task_config])` overloads are deprecated.
- Touch handling follows the existing ESPP BSP pattern and uses the FT5x06
  controller plus the ESPP interrupt and LVGL touch-input helpers.

## Example

The [example](./example) shows how to initialize the display, register touch
input, play a click sound through the speaker path, adjust backlight
brightness, rotate the display, and optionally mount the microSD card.
