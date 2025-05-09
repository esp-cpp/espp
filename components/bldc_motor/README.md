# BLDC (Brushless DC) Motor Component

[![Badge](https://components.espressif.com/components/espp/bldc_motor/badge.svg)](https://components.espressif.com/components/espp/bldc_motor)

The `BldcMotor` implements the Field-Oriented Control (FOC) algorithm with
support for multiple transforms to drive voltage (such as Sinusoidal and Space
Vector). It supports the following motion control configurations (which can be
changed dynamically):

* Closed-loop angle
* Closed-loop velocity
* Open-loop angle
* Open-loop velocity

Note: currently the code has some support for Torque control, but that requires
current sense - for which I don't yet have the hardware to support the
development of.

The `BldcMotor` should be configured with a `BldcDriver` and optional `Sensor`
(for angle & speed of the motor), and optional `CurrentSensor` (for measuring
the phase currents of the motor and providing torque control).

## Example

The [example](./example) shows the use of the `BldcMotor` component to drive a
BLDC motor (such as a tiny gimbal motor) using Field-Oriented Control (FOC) in
both open-loop and closed-loop control schemes for both position and velocity
control.

