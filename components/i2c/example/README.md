# I2C Example

This example shows how to use the `I2C` component to communicate with
peripherals on the I2C bus.

It is currently designed to run on a QtPy ESP32, but (by changing the I2C pin
definitions in the main file) can be reconfigured to run on any of the ESP32
chips.

## How to use example

### Hardware Required

This example requires a connection (via I2C) to an I2C device.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

