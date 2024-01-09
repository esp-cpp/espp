# MAX1704X Example

This example shows how to use the MAX1704X driver to control 

## How to use example

### Hardware Required

This example requires a connection (via I2C) to a dev board which has a MAX1704X
battery gauge chip. The example was tested with a QtPy ESP32s3 dev board and a
MAX1704X breakout board from Adafruit, but can be configured (using
`menuconfig`) to run on any ESP board by configuring the I2C pins and selecting
`CUSTOM` hardware.

- [MAX17048 Breakout board from Adafruit](https://www.adafruit.com/product/5580)
- [QtPy ESP32 Pico](https://www.adafruit.com/product/5395)

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output
