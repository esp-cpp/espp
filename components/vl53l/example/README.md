# VL53LXX Example

This example shows the use of the `Vl53l` component to communicate with a VL53L0X
time-of-flight distance sensor.

The example initializes the sensor, configures it, and then reads the distance
from the sensor every 50ms. The distance is then printed to the console.

![CleanShot 2025-03-19 at 08 40 32](https://github.com/user-attachments/assets/93dc9340-8be6-4b9b-b592-958caa26570b)

## Hardware Required

This example is designed to work with any ESP32 which has I2C pins exposed, but
is pre-configured to run on QtPy boards such as the QtPy ESP32 Pico and QtPy
ESP32-S3.

It requires that you have a time of flight distance sensor dev board, such as
the [VL53L0X dev board from Adafruit](https://www.adafruit.com/product/3317).

![image](https://github.com/user-attachments/assets/47f7f64a-ad55-4529-9851-5283fc57bcb8)

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

![CleanShot 2025-03-19 at 08 38 47](https://github.com/user-attachments/assets/63d059ac-41e5-4692-9fc5-f35ebdb455a2)
