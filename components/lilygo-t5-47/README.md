# LilyGo T5 4.7" (ESP32-S3) E-Paper BSP

Board Support Package for the [LilyGo T5 4.7 inch ESP32-S3
e-paper](https://www.lilygo.cc/products/t5-4-7-inch-e-paper-v2-3) board.

The board drives an **ED047TC1** 960x540, 16-level-grayscale e-paper panel over
a parallel bus (the ESP32-S3 `LCD_CAM` peripheral). This BSP wraps the
[epdiy](https://github.com/vroland/epdiy) library, which owns the timing,
waveforms, grayscale rendering and partial-update handling for this exact board.

## Features

- 4.7" 960x540 e-paper display (ED047TC1) via epdiy: init, power on/off, clear,
  a grayscale framebuffer to draw into, and screen updates.
- LVGL 9 integration: an 8-bit grayscale (`L8`) display whose flush writes into
  the epdiy framebuffer, batching each refresh cycle's dirty areas into a single
  panel update. Choose the update mode (`MODE_GC16` for crisp 16-level grayscale
  or a faster mono mode like `MODE_DU`) and force a full refresh to clear
  ghosting.
- Display rotation (`set_rotation()` / `rotate()`): rotates the e-paper and
  resizes the LVGL display to match, and transforms touch coordinates so they
  stay aligned in every orientation.
- BOOT button (GPIO0) via `espp::Interrupt`.
- Capacitive touch (GT911) on the internal I2C bus, registered as an LVGL input
  device so on-screen widgets are tappable. The capacitive home button below the
  display is reported via `home_button_pressed()` / `touchpad_data().btn_state`.
- PCF8563 real-time clock (via the register-compatible `espp::Bm8563` driver).
- BQ27220 battery fuel gauge (`espp::Bq27220`): voltage, current,
  state-of-charge, temperature, capacity, etc.
- PCA9535 I/O expander (`espp::Pca9535`) — the same chip epdiy uses for the
  e-paper power ICs — with an `io48_button_pressed()` convenience for the
  "IO48"-labelled button (on expander pin PC12).
- LoRa radio (SX1262) on the shared SPI bus (`initialize_lora()` / `lora()`),
  using the `espp::Sx126x` driver — same interface as the t-deck / cardputer
  BSPs.
- Frontlight on/off (`set_frontlight()`, BL_EN / GPIO11).
- Power off via `shutdown()`, which puts the BQ25896 PMIC into ship mode
  (disconnects the battery); the PWR button turns the board back on. Only powers
  off when running on battery (not while USB is connected).
- qwiic / STEMMA-QT connector, wired to the internal I2C bus (`qwiic_i2c()`).
- microSD card (SPI) mounted as a FAT filesystem at `/sdcard`.

The internal I2C bus (SDA=39/SCL=40) is created by this BSP and shared with
epdiy's e-paper power ICs, so the board's other I2C peripherals (touch, RTC,
battery gauge, the PCA9535 I/O expander and the qwiic connector) all live on the
same bus. Access it via `internal_i2c()`.

> **The PCA9535 I/O expander is shared with epdiy.** epdiy drives this same chip
> (address 0x20) to sequence the e-paper's high-voltage rails and owns port 1's
> high bits. The BSP's `io_expander()` is created without re-initializing the
> chip; only read inputs or drive pins epdiy does not use, always with
> read-modify-write. The IO48 button lives on expander pin PC12 (port 1, bit 2),
> which epdiy leaves alone.

## Pins

The non-display pins below are taken from the LilyGo *T5 4.7" ePaper S3 PRO*
factory firmware. The display itself is driven entirely by epdiy's
`lilygo_board_s3` board definition.

| Function | Pin |
|----------|-----|
| BOOT button | GPIO0 |
| Internal I2C SDA / SCL | GPIO39 / GPIO40 |
| GT911 touch INT / RST | GPIO3 / GPIO9 |
| PCF8563 RTC IRQ | GPIO2 |
| PCA9535 expander INT | GPIO38 |
| Frontlight (BL_EN) | GPIO11 |
| microSD SCLK / MOSI / MISO / CS | GPIO14 / GPIO13 / GPIO21 / GPIO12 |
| LoRa (SX1262) CS / DIO1 / RST / BUSY | GPIO46 / GPIO10 / GPIO1 / GPIO47 (shared SPI) |

I2C addresses on the internal bus: GT911 touch `0x5D`, PCF8563 RTC `0x51`,
BQ27220 gauge `0x55`, PCA9535 expander `0x20`.

## Notes on e-paper

E-paper is not a color TFT: updates are slow (hundreds of ms), the panel needs
its high-voltage rails powered only during an update, and the waveform is
temperature dependent. Prefer full refreshes (`clear()` / `MODE_GC16`) to clear
ghosting and partial/mono modes for fast incremental updates.

## Example

See `example/` for a minimal bring-up (clear + draw a test pattern). Build with
the ESP-IDF component manager enabled (needed to fetch epdiy):

```
cd example
idf.py set-target esp32s3
idf.py build flash monitor
```
