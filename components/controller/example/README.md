# Controller Example

This example shows how to use the `Controller` component to create various
different controller interface objects, such as:
* Digital input only controller (using the `DigitalConfig`) with only a couple of buttons (such as NES)
* Analog and digital controller (with 1 joystick that has x/y axes) using the `espp::OneshotAdc` class with the Joystick
* Analog controller with 2 joysticks using the `espp::Ads1x15` component to read 4 analog axes over I2C.

## How to use example

NOTE: this example cannot be run as-is. It's designed to be illustrative and to
check various interfaces. To run the example, you should comment out / remove
the code for all tests except the one you wish to run.

### Hardware Required

You may require analog inputs (raw) or an analog expander (ADS1015) if you want
to run those parts of the example. This was tested with the SparkFun Gamepad
Arduino Shield (for the raw input), and the Adafruit Controller PiHat (for the
Ads1015 test).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
