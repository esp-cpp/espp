BLDC Types & Concepts
*********************

The `bldc_types` component holds the shared building blocks used across the BLDC
motor components (`bldc_motor`, `bldc_haptics`, and `bldc_current_sense`):

- the FOC value types (``DqCurrent``, ``DqVoltage``, ``PhaseCurrent``) and the
  FOC math helpers / constants in ``foc_utils.hpp``,
- the control enumerations (``MotionControlType``, ``TorqueControlType``,
  ``FocType``) in ``bldc_types.hpp`` and the ``SensorDirection`` enum,
- the interface concepts in ``bldc_concepts.hpp`` (``DriverConcept``,
  ``SensorConcept``, ``CurrentSensorConcept``, and ``MotorConcept``) plus the
  ``DummyCurrentSense`` default.

Keeping these in a small, dependency-light component lets the other BLDC
components share the same interface contracts without depending on each other
(for example, ``bldc_current_sense`` implements ``CurrentSensorConcept`` without
pulling in ``bldc_motor``).

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/bldc_types.inc
.. include-build-file:: inc/foc_utils.inc
.. include-build-file:: inc/bldc_concepts.inc
.. include-build-file:: inc/sensor_direction.inc
