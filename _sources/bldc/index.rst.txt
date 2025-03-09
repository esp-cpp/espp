BLDC APIs
*********

.. toctree::
    :maxdepth: 1

    bldc_driver
    bldc_motor

These components provide interfaces by which the user can control brushless DC
(BLDC) motors. The driver component(s) implement the low-level voltage / pwm
output to the motor directly, where the motor component(s) implement the
open-loop or closed-loop control algorithms - using the driver.

Code examples for the BLDC API are provided in the `bldc_motor` example folder.
