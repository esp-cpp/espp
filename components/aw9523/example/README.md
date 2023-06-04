# AW9523 Example

This example shows how to use the `Aw9523` component to communicate (via I2C)
with an AW9523 I2C digital IO expander and LED driver.

It is currently designed to run on an ESP32-S3, but (by changing the I2C pin
definitions in the main file) can be reconfigured to run on any of the ESP32
chips.

## How to use example

### Hardware Required

This example requires a connection (via I2C) to a dev board which has an AW9523.
To that chip should be attached some digial inputs (such as buttons) as well as
an RGB LED.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![example output](https://user-images.githubusercontent.com/213467/225363770-77a52ba7-c8ad-4c12-a416-f9718eca3db2.png)

An example video:

https://user-images.githubusercontent.com/213467/225365600-5ad600e9-68d6-4ddc-96d6-f56b72ec1a0e.mp4
