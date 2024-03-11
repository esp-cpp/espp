# Interrupt Example

This example shows how to use the `espp::Interrupt` class to configure
interrupts for one or more GPIO pins.

## How to use example

### Hardware Required

This example is designed to be run on an ESP32 board which has a digital button
connected to GPIO0 (which is the boot pin and has a button atached on most dev
boards). Additionally, the example assumes an input on GPIO12.

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

