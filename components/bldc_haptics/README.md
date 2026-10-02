# BLDC (Brushless DC) Motor Haptics Component

[![Badge](https://components.espressif.com/components/espp/bldc_haptics/badge.svg)](https://components.espressif.com/components/espp/bldc_haptics)

The `BldcHaptics` class is a high-level interface for controlling a BLDC motor
with a haptic feedback loop. It is designed to be used to provide haptic
feedback as part of a rotary input device (the BLDC motor). The component
provides a `DetentConfig` interface for configuring the input / haptic feedback
profile for the motor dynamically with configuration of:

  - Range of motion (min/max position + width of each position)
  - Width of each position in the range (which will be used to calculate the
    actual range of motion based on the number of positions)
  - Strength of the haptic feedback at each position (detent)
  - Strength of the haptic feedback at the edges of the range of motion
  - Specific positions to provide haptic feedback at (detents)
  - Snap point (percentage of position width which will trigger a snap to the
    nearest position)

The component also provides a `HapticConfig` interface for configuring the
haptic feedback loop with configuration of:

  - Strength of the haptic feedback
  - Frequency of the haptic feedback [currently not implemented]
  - Duration of the haptic feedback [currently not implemented]

## Web console

The [Haptics Console](https://esp-cpp.github.io/espp/apps/haptics_console.html)
(`web/haptics_console.html`) drives the USB example over WebUSB. Opened with
`?autoconnect=1&transport=usb&vid=0x1209&pid=...[&serial=...]` (the query the
Device Hub's links carry) it connects on load, without the browser chooser, to
a device the page was already granted; its **auto-reconnect** checkbox (default
on, remembered per origin) reconnects after an unexpected link loss such as a
device reboot or re-plug. Both only see devices the browser already granted to
the page (served over HTTPS, or HTTP on localhost; a `file://` copy loses the
grant on reload).

## Example

The [example](./example) shows the use of the `BldcHaptics` component to drive a
BLDC motor (such as a tiny gimbal motor) as a user input / output device that
provides haptic feedback (such as might be used as a rotary encoder input).

