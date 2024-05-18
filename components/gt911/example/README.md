# GT911 Example

This example shows how to use the GT911 touch controller with ESP32. It is
designed to run on a LilyGo T-Deck (no home button) or the ESP32-S3-BOX-3 (with
home button).

## How to use example

### Hardware Required

LilyGo T-Deck orESP32-S3-BOX-3 (or any other ESP32 board with a GT911 touch controller)

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

![CleanShot 2024-05-18 at 13 58 18](https://github.com/esp-cpp/espp/assets/213467/e4d0c65a-71b8-4a45-a491-26c6630ca4c9)
