# ESP32-TIMER-CAM Example

This example shows how to use the `espp::EspTimerCam` hardware abstraction
component to automatically detect and initialize components on the
ESP32-TimerCam.

It initializes the LED, RTC, I2C, and ADC.

## How to use example

### Hardware Required

This example is designed to run on the ESP32-TimerCam.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT -b 1500000 flash monitor
```

(Replace PORT with the name of the serial port to use.)

Note: the baudrate for the ESP32 TimerCam cannot go up to 2 MBaud, so 1.5MBaud
is recommended.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

