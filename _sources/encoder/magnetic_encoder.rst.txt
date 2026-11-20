Magnetic Encoder Base
*********************

The ``MagneticEncoderBase`` is the shared CRTP base class for the magnetic
angle encoders (:doc:`as5600` and :doc:`mt6701`). It holds all of the machinery
common to those encoders:

* the periodic update loop (raw-count accumulation + velocity estimation),
* the position / velocity accessors (count, radians, degrees, accumulator, RPM),
* and the periodic driver that calls ``update()`` at the configured rate.

The concrete encoder supplies exactly one thing - a ``read()`` that refreshes
the raw count from the sensor - which the base invokes through static (CRTP)
dispatch, so there is no virtual-call overhead even when the update loop runs at
very high frequency (e.g. 1-2 kHz).

The periodic driver is selected at compile time (per encoder, via KConfig /
menuconfig):

* a :cpp:class:`espp::HighResolutionTimer` (default), which has microsecond
  resolution and is the right choice for sub-millisecond update periods, or
* an :cpp:class:`espp::Timer`, which schedules against an absolute wake-up time
  (drift-free and in phase - important for stable velocity / accumulator state)
  but is limited to the FreeRTOS tick resolution.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/magnetic_encoder_base.inc
