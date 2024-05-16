# MotorGo-Mini Example

This example demonstrates how to use the `espp::MotorGoMini` component to
initialize the hardware on the [MotorGo Mini board](https://motorgo.net) which
is connected to two encoders and two BLDC motors. It uses those hardware to
drive the motors and outputs the state as a CSV.

If you press the boot button it will toggle between closed-loop angle control
and closed-loop velocity control.

## How to use example

### Hardware Required

This example requires a MotorGo Mini board, two EncoderGo boards, and two
motors. The MotorGo Mini board should be connected to the two EncoderGo boards
and the motors.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2024-05-16 at 14 10 38](https://github.com/esp-cpp/espp/assets/213467/11bafc1c-4358-4cb0-a2c3-05df0a888c94)

https://github.com/esp-cpp/espp/assets/213467/709e2aa4-84bb-48e6-882d-bad559eaa4f2

![CleanShot 2024-05-16 at 14 22 20](https://github.com/esp-cpp/espp/assets/213467/8bfa7d8c-223b-470b-88af-6e89554e6a66)
