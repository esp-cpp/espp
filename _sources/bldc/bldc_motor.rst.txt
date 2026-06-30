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

Torque control is supported in two forms. Voltage-mode torque
(``TorqueControlType::VOLTAGE``) requires no current sensing and is fully
supported. Current-feedback torque (``TorqueControlType::DC_CURRENT`` and
``TorqueControlType::FOC_CURRENT``) closes a current loop and therefore requires
a current sensor implementing the ``CurrentSensorConcept`` (for example
``espp::CurrentSense`` from the ``bldc_current_sense`` component). The
current-feedback modes are experimental: they depend on accurate,
PWM-synchronized current sampling and per-board calibration, and must be
validated on hardware.

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
