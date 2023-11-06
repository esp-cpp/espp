# BM8563 Example

This example shows how to use the BM8563 RTC driver to set and get the time.

## How to use example

### Hardware Required

This example can be run on any ESP32 board, the BM8563 is connected to the ESP32
via I2C. This example uses the following I2C configuration:
- SDA GPIO 21
- SCL GPIO 22
- Frequency 400KHz

Which is a working configuration on the ESP32 TimerCam.

Note: on the ESP32 TimerCam, you'll need to use a flash baudrate <= 1500000.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

