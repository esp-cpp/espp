# CST816 Example

This example shows how to use the CST816 touch controller with ESP32. It is
designed to run on a [Matouch Rotary
Display](https://wiki.makerfabs.com/MaTouch_ESP32_S3_Rotary_IPS_Display_1.28_GC9A01.html).

## How to use example

### Hardware Required

The [Matouch Rotary
Display](https://wiki.makerfabs.com/MaTouch_ESP32_S3_Rotary_IPS_Display_1.28_GC9A01.html)
is required for this example.

If you have other hardware with CST816, you can run menuconfig, and select the
`HARDWARE_CUSTOM` option. and then select which pins are connected to the
CST816.

### Configure

``` 
idf.py menuconfig
```

Set the hardware configuration for the example.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2024-07-08 at 21 57 00](https://github.com/esp-cpp/espp/assets/213467/c275598a-0759-4d40-b22b-224d2197be8f)
