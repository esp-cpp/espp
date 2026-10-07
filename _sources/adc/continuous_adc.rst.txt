Continuous ADC
**************

The `ContinuousAdc` provides a mechanism for high-frequency, continuous,
deterministic sampling of analog voltages for multiple channels (potentially
across multiple ADC units, depending on the ESP32 chip used). It does this be
enabling the continuous ADC DMA mode and then running its own task which
retrieves the data and filters it. When the user calls `get_mv(adc_channel_t)`,
it simply returns the most recent filtered value for that channel, if it was
configured.

Conversion Modes (multiple units)
---------------------------------

On chips whose digital controller supports both ADC units in DMA mode (e.g.
ESP32-S2 / ESP32-P4), channels from both units can be sampled by one
`ContinuousAdc`. Two conversion modes control how the hardware schedules them:

* ``ADC_CONV_ALTER_UNIT`` (the derived default when the configured channels
  span both units): the controller performs one conversion per trigger,
  alternating between the units. Every channel is sampled evenly at
  approximately ``sample_rate_hz`` regardless of how the channels are split
  across the units. Use this for general multi-channel monitoring - e.g.
  several independent sensors that happen to be wired to pins on different
  units:

  .. code-block:: c++

     // joystick axes on ADC1 + battery divider on ADC2, each sampled ~1 kHz
     espp::ContinuousAdc adc({.sample_rate_hz = 1000,
                              .channels = {joy_x, joy_y, vbat}});

* ``ADC_CONV_BOTH_UNIT``: both units convert simultaneously (in lockstep) on
  every trigger. Choose it explicitly when the *relative timing* of two
  signals matters - e.g. sampling a voltage (on ADC1) and a current (on ADC2)
  at the same instant to compute instantaneous power, or capturing
  phase-matched sensor pairs:

  .. code-block:: c++

     espp::ContinuousAdc adc({.sample_rate_hz = 1000,
                              .channels = {v_sense, i_sense},
                              .convert_mode = ADC_CONV_BOTH_UNIT});

  Note that with ``ADC_CONV_BOTH_UNIT`` each unit converts on every trigger,
  so the effective per-channel rate is higher than ``sample_rate_hz`` (twice,
  for a one-channel-per-unit configuration); ``get_rate()`` reports the
  actual rate.

.. ------------------------------- Example -------------------------------------

.. toctree::

   adc_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/continuous_adc.inc
