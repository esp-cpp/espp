# BDC (Brushed DC) Motor Driver Component

[![Badge](https://components.espressif.com/components/espp/bdc_driver/badge.svg)](https://components.espressif.com/components/espp/bdc_driver)

The `BdcDriver` component wraps the `ESP MCPWM Peripheral
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html>`_
to drive a brushed DC motor through a dual-PWM H-bridge style interface.

It provides:

- two MCPWM outputs per motor channel
- direct duty-cycle control for each output
- a signed speed helper where positive commands drive output A, negative
  commands drive output B, and zero disables both outputs
- optional driver-enable GPIO handling
- readback of the last commanded normalized and raw duty values for logging and
  debugging

## Example

The [example](./example) shows a simple sinusoidal speed sweep using two GPIOs.
Update the GPIO assignments in the example to match your hardware before
running it on a real motor driver.
