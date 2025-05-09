# BLDC (Brushless DC) Motor Driver Component

[![Badge](https://components.espressif.com/components/espp/bldc_driver/badge.svg)](https://components.espressif.com/components/espp/bldc_driver)

The `BldcDriver` component wraps around the `ESP MCPWM Peripheral
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html>`_
to provide full 6 PWM control over a 3 phase brushless dc motor.

It is designed to be used by the `espp::BldcMotor` class. Code examples for the
`espp::BldcDriver` can be found in the `bldc_motor` component example.
