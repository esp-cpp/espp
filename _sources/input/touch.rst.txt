Touch Interface
***************

The `touch` component provides the shared `TouchPoint`, `TouchState`,
`TouchpadData`, and `ITouchDevice` types used by ESPP touch controller drivers
and BSPs.

It also provides the type-erasure helpers used to store any touch driver behind
a single runtime handle:

- ``espp::TouchDriverConcept`` — C++23 concept satisfied by any touch driver
  exposing ``update()``, ``get_touch_point()``, and ``get_home_button_state()``.
- ``espp::ITouchDriver`` — abstract type-erased interface backed by the concept.
- ``espp::TouchDriverAdapter<T>`` — concept-constrained adapter wrapping any
  concrete driver (e.g. ``Gt911`` or ``St7123Touch``) behind ``ITouchDriver``.
- ``espp::make_touch_driver(driver)`` — convenience factory returning a
  ``std::shared_ptr<ITouchDriver>``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   touch_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/touch.inc
