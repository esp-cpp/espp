Motor (BDC + BLDC) APIs
***********************

.. toctree::
    :maxdepth: 1

    bdc_driver
    bldc_driver
    bldc_motor

These components provide interfaces by which the user can control brushed DC (BDC) and
brushless DC (BLDC) motors. The driver component(s) implement the low-level
voltage / pwm output to the motor directly, where the motor component(s)
implement the open-loop or closed-loop control algorithms using the driver.

Code examples for the motor-driver APIs are provided in the `bdc_driver` and
`bldc_motor` example folders.
