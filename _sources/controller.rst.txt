Controller APIs
***************

The `Controller` class provides a convenience for reading multiple GPIOs at once
and mapping their state to common controller buttons. It can optionally be
configured to support joystick select, as well as to convert analog joystick
values into digital directional values (up/down/left/right). It can also be used
for just a subset of the buttons, should you wish to do so, by providing the
GPIO configuration for the unused buttons to be -1.

.. ------------------------------- Example -------------------------------------

.. toctree::

   controller_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/controller.inc
