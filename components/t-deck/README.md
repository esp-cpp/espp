# LilyGo T-Deck Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/t-deck/badge.svg)](https://components.espressif.com/components/espp/t-deck)

The LilyGo T-Deck is a development board for the ESP32-S3 module. It features a
touchscreen display, keyboard, trackball, audio, and expansion headers.

The `espp::TDeck` component provides a singleton hardware abstraction for
initializing the touch, display, keyboard, trackball, audio, and micro-SD card
subsystems.

## Supported Features

- ST7789 LCD driven through the shared `espp::Spi` + `SpiPanelIo` path
- GT911 capacitive touch input with LVGL integration helpers
- T-Keyboard input and trackball pointer input
- I2S speaker output with software volume / mute control
- microSD mounting over SDSPI on the shared SPI host
- Backlight brightness control and exposed peripheral helpers

## Example

The [example](./example) shows how to use the `espp::TDeck` hardware abstraction
component to initialize the major subsystems on the LilyGo T-Deck and interact
with the display, touch panel, keyboard, trackball, audio path, and optional
microSD card.
