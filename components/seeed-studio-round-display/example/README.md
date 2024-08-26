# Seeed Studio Round Display Example

This example shows how to use the `espp::SsRoundDisplay` hardware abstraction
component to initialize the hardware components on the [Seeed Studio Round
Display](https://wiki.seeedstudio.com/get_start_round_display/) when used with
either the XIAO S3 or the QtPy ESP32S3.

It initializes the touch and display subsystems. It reads the touchpad state and
each time you touch the scren it uses LVGL to draw a circle where you touch.

## How to use example

### Hardware Required

This example is designed to run on either the QtPy ESP32S3 or the XIAO S3,
though the `espp::SsRoundDisplay` can be used with custom hardware.

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

