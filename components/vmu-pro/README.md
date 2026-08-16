# VMU Pro Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/vmu-pro/badge.svg)](https://components.espressif.com/components/espp/vmu-pro)

The [VMU Pro](https://8bitmods.com/vmupro-handheld-visual-memory-card-for-dreamcast-classic-white/)
by [8BitMods](https://8bitmods.com) is an ESP32-S3 based replacement for the
Sega Dreamcast Visual Memory Unit (VMU) which doubles as a standalone handheld
gaming device. It features:

* 1.5" 240x240 IPS TFT color display
* D-pad, A, B, Mode, Power, and Bottom buttons
* Mono speaker (I2S amplifier)
* micro-SD card slot (mounted at `/sdcard`)
* Rechargeable battery with USB-C charging

The `espp::VmuPro` component provides a singleton hardware abstraction for
initializing the display, buttons, audio, and uSD card subsystems.

> **Warning:** The GPIO assignments in this component are UNVERIFIED
> placeholders. The VMU Pro's schematic is not publicly available, so the pin
> numbers in `include/vmu-pro.hpp` must be corrected against real hardware or
> vendor documentation before the component will function on the device. The
> peripheral set itself (display size and color format, button list, audio
> capabilities, uSD card) comes from the official
> [VMU Pro SDK](https://github.com/AppCakeLtd/vmupro-sdk) and is accurate.

## Example

The [example](./example) shows how to use the `espp::VmuPro` hardware
abstraction component to initialize the components on the VMU Pro and build a
simple LVGL UI (via a `Gui` class) driven by the buttons.
