# MotorGo-Mini Example

This example demonstrates how to use the `espp::MotorGoMini` component to
initialize the hardware on the [MotorGo Mini board](https://motorgo.net) which
is connected to two encoders and two BLDC motors. It uses those hardware to
drive the motors and outputs the state as a CSV.

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

