# BLDC Current Sense

[![Badge](https://components.espressif.com/components/espp/bldc_current_sense/badge.svg)](https://components.espressif.com/components/espp/bldc_current_sense)

The `bldc_current_sense` component provides `espp::CurrentSense`, a field-oriented
control (FOC) current sensor for BLDC motors. It implements the
`CurrentSensorConcept` used by `espp::BldcMotor`, so it can be supplied as a
motor's current sense to enable current-feedback torque control
(`TorqueControlType::DC_CURRENT` and `TorqueControlType::FOC_CURRENT`).

`espp::CurrentSense` is intentionally decoupled from any specific ADC. You
provide a `read_phase_currents` callback that returns the most recently sampled
phase currents (in amps). The class then:

- subtracts the per-phase zero-current offset captured during `driver_align()`,
- applies any per-phase gain-sign correction,
- reconstructs an unmeasured phase (return `NAN` for it) assuming `Ia + Ib + Ic = 0`,
- runs the Clarke + Park transforms to produce the d/q currents
  (`get_foc_currents()`) and the signed DC current magnitude (`get_dc_current()`).

## PWM-synchronized sampling

Accurate low-side current sensing requires sampling while the low-side FETs are
conducting (the PWM center). Use
`espp::BldcDriver::register_pwm_sample_callback()` to trigger your ADC read at
the timer "empty" (TEZ) event, and have your `read_phase_currents` callback
return the latest sampled values.

## Status

> **Experimental.** Zero-current offset calibration is implemented, but automatic
> phase-ordering / gain-sign discovery is not — set `phase_gain_signs` in the
> `Config` if a sense channel is inverted. Current-mode torque control depends on
> accurate, PWM-synchronized sampling and per-board tuning and must be validated
> on hardware.

## Example

The example builds a software model of the analog front-end (no motor hardware
required) to exercise the calibration + Clarke/Park pipeline, and statically
verifies that `espp::CurrentSense` satisfies the `CurrentSensorConcept`.
