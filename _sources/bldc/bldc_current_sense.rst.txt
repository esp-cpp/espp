BLDC Current Sense
******************

The `CurrentSense` class provides a field-oriented control (FOC) current sensor
for BLDC motors. It implements the `CurrentSensorConcept` used by `BldcMotor`, so
it can be supplied as a motor's current sense to enable current-feedback torque
control (`TorqueControlType::DC_CURRENT` and `TorqueControlType::FOC_CURRENT`).

`CurrentSense` is decoupled from any specific ADC: the user provides a
`read_phase_currents` callback that returns the most recently sampled phase
currents (in amps). The class subtracts the per-phase zero-current offset
captured during `driver_align()`, applies optional per-phase gain-sign
correction, reconstructs an unmeasured phase (return ``NAN`` for it) assuming
``Ia + Ib + Ic = 0``, and runs the Clarke + Park transforms to produce the d/q
currents (`get_foc_currents()`) and the signed DC current magnitude
(`get_dc_current()`).

For accurate low-side current sensing the samples must be taken while the
low-side FETs conduct (the PWM center). Use
`BldcDriver::register_pwm_sample_callback()` to trigger the ADC read at the
timer "empty" (TEZ) event and have the `read_phase_currents` callback return the
latest result.

.. note::

   This component is **experimental**. Zero-current offset calibration is
   implemented, but automatic phase-ordering / gain-sign discovery is not (set
   ``phase_gain_signs`` in the ``Config`` if a sense channel is inverted).
   Current-mode torque control depends on accurate, PWM-synchronized sampling and
   per-board tuning and must be validated on hardware.

.. ------------------------------- Example -------------------------------------

.. toctree::

   bldc_current_sense_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/current_sense.inc
