# QwiicNES Example

This example demonstrates the use of the QwiicNES component to read the state of
a NES controller and print it to the serial console.

## How to use example

### Hardware Required

This example requires a connection (via I2C) to a QwiicNES board and a NES
controller.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

<img width="795" alt="CleanShot 2023-11-05 at 11 07 00@2x" src="https://github.com/esp-cpp/espp/assets/213467/58ca2068-8e94-4c91-9a85-0b3f15528d3d">
