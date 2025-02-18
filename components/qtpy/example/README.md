# QtPy Example 

This example shows the use of the `QtPy` class to perform hardware
initialization for the QtPy ESP32 Pico and the QtPy ESP32-S3.

## How to use example

### Hardware Required

This example is designed to be run on either a [QtPy ESP32
Pico](https://www.adafruit.com/product/5395) or a [QtPy
ESP32-S3](https://www.adafruit.com/product/5426). It uses the QtPy's on-board
button and NeoPixel. It will also scan for any devices that are present on the
QtPy's QWIIC I2C port.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output


