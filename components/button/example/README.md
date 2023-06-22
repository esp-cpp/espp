# Button Example

This example shows how to use the `espp::Button` class to handle input events
from a physical button connected to an ESP32 GPIO pin.

## How to use example

### Hardware Required

This example is designed to be run on an ESP32 board which has a digital button
connected to GPIO2 of the ESP32, with the button having a hardware pull-down
resistor so that its resting state is logic low and its active / pressed state
is logic high.

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

![CleanShot 2023-06-22 at 16 23 59](https://github.com/esp-cpp/espp/assets/213467/cc2a131c-7c78-4ec7-8bff-1f629137a41b)
