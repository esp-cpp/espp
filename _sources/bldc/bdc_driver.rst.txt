BDC Driver
**********

The `BdcDriver` component wraps the `ESP MCPWM Peripheral
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html>`_
to provide dual-PWM control for brushed DC motors and H-bridge style drivers.

It supports direct duty-cycle control of the two motor outputs as well as a
signed speed helper that maps positive commands to output A and negative
commands to output B.

.. ------------------------------- Example -------------------------------------

.. toctree::

   bdc_driver_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/bdc_driver.inc
