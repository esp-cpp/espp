# Tla2528 Example

This example shows the use of the `Tla2528` component to communicate with an
TLA2528 I2C analog to digital converter using the I2C peripheral of the ESP32.

It uses the `task` component to periodically read two channels of analog data
and one channel of digital input from the TLA2528, set one channel of digital
output, and print the data in CSV format using the `format` component.

## How to use example

### Hardware Required

To run this example, you need:
* [QtPy ESP32S3](https://www.adafruit.com/product/5426)
* [TLA2528]()
* [Adafruit Thumb Joystick](https://www.adafruit.com/product/512)
* 1 kOhm resistor (between Joystick Select and 3.3V)
* Some dupont wires
* QwiiC cable (JST-SH to Dupont)

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

