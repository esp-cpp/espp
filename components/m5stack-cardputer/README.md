# M5Stack Cardputer Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/m5stack-cardputer/badge.svg)](https://components.espressif.com/components/espp/m5stack-cardputer)

The M5Stack Cardputer (K132) and Cardputer ADV are card-sized computers based
on the ESP32-S3 StampS3 module, with a 56-key QWERTY keyboard, a 1.14" 240x135
IPS display, a mono speaker, a microphone, a micro-SD card slot, an IR
transmitter, a Grove port, and a WS2812 RGB LED.

The `espp::M5StackCardputer` component supports **both variants with the same
API**, detecting the board at runtime (see `variant()`), and provides a
singleton hardware abstraction for initializing and using the subsystems:

- **Display**: 1.14" 240x135 IPS TFT (ST7789V2) over SPI @ 40 MHz, with LVGL
  integration via `espp::Display` and PWM backlight brightness control.
- **Keyboard**: on the original, the 56-key matrix (scanned through a 74HC138
  3-to-8 demultiplexer) is polled by a scanner task; on the ADV the same 56
  keys are read from the TCA8418 I2C keyboard controller's event FIFO. Either
  way, a callback is invoked once per key state change with the key's matrix
  position, resolved character (shift-aware), fn-layer special key (F1-F12,
  arrows, esc, delete), and modifier state (fn / shift / ctrl / opt / alt).
- **Audio output**: mono speaker with `play_audio()`, runtime sample-rate
  control, and software volume / mute. On the original the I2S stream drives
  an NS4168 amplifier directly; on the ADV it goes through an ES8311 codec
  (initialized automatically) into an NS4150B amplifier.
- **Microphone**: a recording task delivers 16-bit mono samples to a
  callback. Original: SPM1423 PDM microphone; ADV: analog MEMS microphone via
  the ES8311 codec's ADC.
- **uSD card**: SPI-mode micro-SD mounted at `/sdcard`.
- **RGB LED**: the StampS3's WS2812 via `espp::Neopixel`.
- **Battery**: battery voltage measurement (2:1 divider into ADC1).
- **G0 (BOOT) button**: via `espp::Interrupt` with a callback.
- **IR transmitter / Grove port**: pin accessors for use with your own RMT /
  I2C drivers.
- **Internal I2C bus** (ADV only): accessor for the bus hosting the TCA8418
  (0x34), ES8311 (0x18), and the ADV's BMI270 IMU (0x68) - the IMU can be
  used with the espp `bmi270` component.

> [!NOTE]
> The speaker and the microphone share I2S pins (GPIO 43 word-select / PDM
> clock on the original; the BCK/WS pair on the ADV), so they cannot be used
> at the same time; initializing one while the other is active will fail.
> This matches the boards' hardware design.

## Example

The [example](./example) shows how to use the `espp::M5StackCardputer` hardware
abstraction component to initialize the display (with a `Gui` class built on
LVGL), type text with the keyboard, play key-click sounds on the speaker, cycle
the RGB LED color, and monitor the battery voltage.
