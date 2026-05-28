# Smart Panlee SC01 Plus BSP Example

This example demonstrates the core functionality of the `espp::SmartPanleeSc01Plus`
BSP component on the Smart Panlee SC01 Plus touchscreen display module.

<img width="1515" height="1141" alt="image" src="https://github.com/user-attachments/assets/1d6542f3-d064-4a2c-8913-8a7b1782d703" />

## Features Demonstrated

- ST7796 display initialization over the ESP32-S3 8-bit parallel LCD bus
- FT5x06 touch handling with LVGL integration
- I2S speaker playback with a bundled touch-click sound
- Backlight brightness control
- Optional microSD mounting and filesystem inspection
- Published peripheral pin maps for I2S, RS-485, and external GPIOs

## Hardware Required

- Smart Panlee SC01 Plus / WT32-SC01-PLUS compatible board
- USB-C cable for programming and power
- microSD card (optional)

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output and verify that touching the screen draws circles and plays the
embedded click sound:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)
