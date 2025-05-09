# Drv2605 I2C Haptic Motor Driver Component

[![Badge](https://components.espressif.com/components/espp/drv2605/badge.svg)](https://components.espressif.com/components/espp/drv2605)

The `DRV2605` haptic motor driver component allows the user to configure and
play custom or preconfigured haptic feedback sequences via a serial interface
such as I2C. It supports directly driving ECM (eccentric rotating mass) and LRA
(linear resonant actuator) type haptic motors. It also supports fully custom
waveforms (e.g. via using the audio, pwm / analog functions) as well as a preset
library of 123 different haptic waveforms which can be played in sequences of up
to 8 waveforms.

If you wire up the trigger / in line and configure it via I2C, then you can
directly drive the haptic motors with audio waveforms.

## Example

The [example](./example) shows the use of the `drv2605` component to communicate with and
control a DRV2605 I2C haptic motor driver for linear resonant actuator (LRA) and
eccentric rotating mass (ERM) haptic motors.

