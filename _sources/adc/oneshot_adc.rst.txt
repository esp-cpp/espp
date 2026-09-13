Oneshot ADC
***********

The `OneshotAdc` allows the user a simple, low-resource way to sporadically (and
with moderate frequency needs) measure an analog voltage for multiple channels
on a single ADC UNIT. It does not start or manage any tasks and does not perform
any filtering on the data. Each time the user calls `read_raw(adc_channel_t)` or
`read_mv(adc_channel_t)`, it block and trigger an analog read for the associated
channel (if it was configured to do so).

.. ------------------------------- Example -------------------------------------

.. toctree::

   adc_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/oneshot_adc.inc
