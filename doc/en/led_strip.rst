LED Strip APIs
**************

The `LedStrip` component provides APIs to control LED strips. It supports
various LED strip types, such as WS2812, WS2811, WS2813, SK6812, APA102, etc.
You can use it directly with an SPI driver to talk to APA102 LED strips, or you
can use it with a RMT driver (such as the `Rmt` component) to talk to WS2812,
WS2811, WS2813, SK6812, etc. LED strips.

.. ------------------------------- Example -------------------------------------

.. toctree::

   led_strip_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/led_strip.inc
