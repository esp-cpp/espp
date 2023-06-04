# Ads1x15 Example

This example shows the use of the `Ads1x15` component to communicate with an
ADS1015 I2C analog to digital converter using the I2C peripheral of the ESP32.

It uses the `task` component to periodically read two channels of analog data
from the ADS1015 and print them in CSV format using the `format` component.

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![example_output](https://user-images.githubusercontent.com/213467/225378943-a1aabb6a-c12c-4c91-9fcc-9e2d76555e3e.png)
