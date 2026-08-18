# Magnetic Encoder Base

[![Badge](https://components.espressif.com/components/espp/magnetic_encoder/badge.svg)](https://components.espressif.com/components/espp/magnetic_encoder)

The `espp::MagneticEncoderBase` class is the shared CRTP base class for the espp
magnetic angle encoders (`espp::As5600` and `espp::Mt6701`). It holds all of the
machinery common to those encoders:

* the periodic update loop (raw-count accumulation + velocity estimation),
* the position / velocity accessors (count, radians, degrees, accumulator, RPM),
* and the periodic driver that calls `update()` at the configured rate.

The concrete encoder supplies exactly one thing - a `read()` that refreshes the
raw count from the sensor - which the base invokes through static (CRTP)
dispatch, so there is no virtual-call overhead even when the update loop runs at
very high frequency (e.g. 1-2 kHz).

The periodic driver is selected at compile time (per encoder, via KConfig /
menuconfig):

* an `espp::HighResolutionTimer` (default), which has microsecond resolution and
  is the right choice for sub-millisecond update periods, or
* an `espp::Timer`, which schedules against an absolute wake-up time (drift-free
  and in phase - important for stable velocity / accumulator state) but is
  limited to the FreeRTOS tick resolution.

This component is a building block; it is not instantiated directly. See the
`as5600` and `mt6701` components for concrete encoders that derive from it.
