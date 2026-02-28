Continuous ADC
**************

The `ContinuousAdc` provides a mechanism for high-frequency, continuous,
deterministic sampling of analog voltages for multiple channels (potentially
across multiple ADC units, depending on the ESP32 chip used). It does this be
enabling the continuous ADC DMA mode and then running its own task which
retrieves the data and filters it. When the user calls `get_mv(adc_channel_t)`,
it simply returns the most recent filtered value for that channel, if it was
configured.

.. ------------------------------- Example -------------------------------------

.. toctree::

   adc_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/continuous_adc.inc
