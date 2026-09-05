Joystick APIs
*************

Joystick
--------

The `Joystick` class provides a wrapper around a 2-axis analog joystick, with an
associated reader function for grabbing the raw values. When the joystick
`update()` is called, the raw values are mapped into the range [-1,1] for each
axis according to the configuration provided.

Code examples for the task API are provided in the `joystick` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   joystick_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/joystick.inc
