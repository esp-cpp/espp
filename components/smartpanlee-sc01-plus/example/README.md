# Smart Panlee SC01 Plus BSP Example

This example demonstrates the core functionality of the `espp::SmartPanleeSc01Plus`
BSP component on the Smart Panlee SC01 Plus touchscreen display module.

<img width="1515" height="1141" alt="image" src="https://github.com/user-attachments/assets/1d6542f3-d064-4a2c-8913-8a7b1782d703" />

## Features Demonstrated

- ST7796 display initialization over the ESP32-S3 8-bit parallel LCD bus
- FT5x06 touch handling with LVGL integration
- Foreground custom-drawn touch trail with bounded invalidation for smooth redraws
- I2S speaker playback with a bundled touch-click sound, plus an on-screen
  audio row (bottom-left) to play the sound, toggle mute, and adjust the
  volume
- Backlight brightness control
- Optional microSD mounting and filesystem inspection
- Published peripheral pin maps for I2S, RS-485, and external GPIOs
- Rotation-aware UI layout with wrapped instructions that stay on-screen in portrait/landscape

## Configuration notes

This example raises some espp BSP task stack sizes above their component
defaults via `sdkconfig.defaults`, because the example does more work in
those tasks than the defaults assume:

- **Interrupt / touch task stack (`CONFIG_SMARTPANLEE_SC01_PLUS_INTERRUPT_STACK_SIZE` = 8192, up from the 4 KB
  BSP default).** The interrupt task services the touch controller over
  I2C; if an I2C transaction errors, the error is logged through espp's
  `fmt`-based (colorized) logger, whose formatting path needs several KB of
  stack. With only 4 KB this can overflow and corrupt memory. If you base
  your own project on this example (or reproduce its functionality), keep
  this override in your `sdkconfig` / `sdkconfig.defaults`.

## Hardware Required

- Smart Panlee SC01 Plus / WT32-SC01-PLUS compatible board
- USB-C cable for programming and power
- microSD card (optional)

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output and verify that touching the screen draws circles and plays the
embedded click sound. The refresh button rotates the display and the label will
reflow to stay within the visible screen width:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)
