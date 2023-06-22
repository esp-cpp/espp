Button APIs
***********

The `Button` class provides a simple wrapper around the ESP-IDF gpio API to
provide a simple interface for reading the state of a button. The class sets the
GPIO pin to input mode and installs an ISR to detect button presses. The ISR
sends data to the `Button`'s queue which is read by the `Button`'s task. The
task periodically checks the queue for new data and updates the button state
accordingly, additionally calling the provided callback function if one was
registered.

Note: the data passed to the callback function is a `espp::Button::Event` object
which contains the button's state and the gpio associated with the button.

Additionally, users can check the state of the button by calling the
`is_pressed` method.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/button.inc
