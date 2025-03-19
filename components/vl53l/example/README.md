# VL53LXX Example

This example shows the use of the `Vl53l` component to communicate with a VL53L0X
time-of-flight distance sensor.

The example initializes the sensor, configures it, and then reads the distance
from the sensor every 50ms. The distance is then printed to the console.

## Hardware Required

This example is designed to work with any ESP32 which has I2C pins exposed, but
is pre-configured to run on QtPy boards such as the QtPy ESP32 Pico and QtPy
ESP32-S3.

It requires that you have a time of flight distance sensor dev board, such as
the [VL53L0X dev board from Adafruit](https://www.adafruit.com/product/3317).

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

