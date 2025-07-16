BLDC Motor
**********

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

.. ------------------------------- Example -------------------------------------

.. toctree::

   bldc_motor_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/bldc_motor.inc
.. include-build-file:: inc/bldc_types.inc
.. include-build-file:: inc/sensor_direction.inc
