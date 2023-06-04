# Joystick Example

This example shows the use of the `Joystick` class to manage the input from and
perform mapping / calibration of joystick data from an analog joystick.

![image](https://user-images.githubusercontent.com/213467/226445048-5e564333-2b53-4592-aac0-f8aa3137301e.png)

## How to use example

### Hardware Required

This example is designed to be used with an analog joystick directly connected
to the ESP ADC 2, channels 8 and 9.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![output](https://user-images.githubusercontent.com/213467/226444494-4d94b212-c9d5-4638-9343-abc21e6f8689.png)
