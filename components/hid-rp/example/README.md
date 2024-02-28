# HID-RP Example

This example shows how to use the
[`hid-rp`](https://github.com/intergatedcircuits/hid-rp) library which is
bundled into the hid-rp component within espp.

It provides an example of a somewhat configurable HID Gamepad using the
`esppp::GamepadReport<>` template class.

## How to use example

### Hardware Required

This example should run on any ESP32s3 development board as it requires no
peripheral connections.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2024-02-28 at 16 38 11](https://github.com/esp-cpp/espp/assets/213467/d86a5977-5db1-44bc-9df9-fcbb751392a5)
