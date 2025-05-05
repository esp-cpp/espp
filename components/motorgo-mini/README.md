# MotorGo-Mini Board Support Package (BSP) Component

The MotorGo Mini is a small, low-cost, low-power motor controller that can be
used to control a two motors.

https://motorgo.net

It's pretty sweet and the component provides the implementation of the two
channel FOC motor controller, along with other peripheral classes such as the
ADC, LEDs, and I2C.

## Example

This example demonstrates how to use the `espp::MotorGoMini` component to
initialize the hardware on the [MotorGo Mini board](https://motorgo.net) which
is connected to two encoders and two BLDC motors. It uses those hardware to
drive the motors and outputs the state as a CSV.

