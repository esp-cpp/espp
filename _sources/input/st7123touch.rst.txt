ST7123 Touch Controller
***********************

The `St7123Touch` class provides an interface to the capacitive touch controller
integrated in the **Sitronix ST7123** TDDI (Touch and Display Driver Integration)
chip.

The ST7123 combines a MIPI-DSI display driver and a multi-touch capacitive
controller in a single IC. This driver accesses the touch side over I2C
(default address **0x55**).

.. note::

   The ST7123's touch engine is enabled by the **LCD_RST** pulse issued during
   display initialization — do *not* toggle the ``TP_RST`` line used by
   standalone controllers such as the GT911, as this can take the touch I2C
   endpoint offline.

``St7123Touch`` satisfies the ``espp::TouchDriverConcept``, so it can be wrapped
in the type-erased ``espp::ITouchDriver`` interface via
``espp::make_touch_driver()``. These shared type-erasure helpers now live in the
:doc:`touch` component and are reused by every espp touch driver and BSP:

- ``espp::TouchDriverConcept`` — C++23 concept satisfied by any touch driver
  exposing ``update()``, ``get_touch_point()``, and ``get_home_button_state()``.
- ``espp::ITouchDriver`` — abstract type-erased interface backed by the concept.
- ``espp::TouchDriverAdapter<T>`` — concept-constrained adapter wrapping any
  concrete driver behind ``ITouchDriver``.
- ``espp::make_touch_driver(driver)`` — convenience factory returning a
  ``std::shared_ptr<ITouchDriver>``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   st7123touch_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/st7123touch.inc
