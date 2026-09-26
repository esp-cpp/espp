LED APIs
********

LED
---

The LED provides a convenient and thread-safe wrapper around the `ESP-IDF LEDC
perhipheral
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html#led-control-ledc>`_.

It allows for both instant and hardware-based timed changing (fading) of duty cycle (in
floating point percent [0,100]).

.. ------------------------------- Example -------------------------------------

.. toctree::

   led_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/led.inc
