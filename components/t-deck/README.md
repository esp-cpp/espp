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
- Audio through the [`espp::Audio`](../audio) component, set up with
  `initialize_audio(AudioConfig)` (each direction can be turned off to save
  its I2S DMA ring and task):
  - I2S speaker output (MAX98357A) with software volume / mute control
    (`play_audio()`, `volume()`, `mute()`, `audio_sample_rate()`)
  - Dual microphone recording through the ES7210 ADC on its own I2S bus (a
    4-slot TDM frame, of which MIC1 and MIC3 are delivered as 16-bit stereo)
    at its own sample rate (`AudioConfig::recording_sample_rate`), via
    `start_audio_recording()`, with adjustable analog gain via
    `microphone_volume()`
  - `audio()` / `microphone_audio()` return the two `espp::Audio` instances
    for anything else; the older `initialize_sound()` /
    `initialize_microphone()` remain as deprecated wrappers
- microSD mounting over SDSPI on the shared SPI host
- Backlight brightness control and exposed peripheral helpers

## Example

The [example](./example) shows how to use the `espp::TDeck` hardware abstraction
component to initialize the major subsystems on the LilyGo T-Deck and interact
with the display, touch panel, keyboard, trackball, audio path, and optional
microSD card.
