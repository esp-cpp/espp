# Wrover-Kit Example

This example shows how to use the `espp::WroverKit` hardware abstraction component
to initialize the ESP32-WROVER-KIT.

It initializes the display subsystem and periodically draws 10 circles to the
screen before clearing and starting over using LVGL.

## How to use example

### Hardware Required

This example is designed to run on the ESP32-WROVER-KIT.

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

![CleanShot 2024-07-05 at 23 57 31@2x](https://github.com/esp-cpp/espp/assets/213467/4882ea59-19d5-43a8-aa74-99d36de6ab32)
