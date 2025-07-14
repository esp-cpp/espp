# Waveshare ESP32-S3 TouchLCD Example

This example shows how to use the `espp::WsS3Touch` hardware abstraction
component to automatically detect and initialize components on the Waveshare
ESP32-S3 TouchLCD board, and use the LVGL graphics library to draw on the
display. It also plays a sound when you touch the screen using the buzzer on the
system, and it changes the frequency of the sound based on the position of the
touch.

## How to use example

### Hardware Required

This example is designed to run on the Waveshare ESP32-S3 TouchLCD board.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output


