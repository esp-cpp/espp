Button APIs
***********

The `Button` class provides a simple wrapper around the ESP-IDF gpio API to
provide a simple interface for reading the state of a button. The class sets the
GPIO pin to input mode and installs an ISR to detect button presses. The ISR sends
data to the `Button`'s queue which is read by the `Button`'s task. The task
periodically checks the queue for new data and updates the button state
accordingly, additionally publishing a message to the `Button`'s topic when the
button state changes via the `EventManager`.

Note: the data published to the `Button`'s topic is a `espp::Button::Event`
object which contains the button's state and the gpio associated with the
button. It is serialized using the `espp::serialization` component, which is a
wrapper around the `alpaca` serialization library. See the example code for
deserializing the event data.

Additionally, users can check the state of the button by calling the `is_pressed`
method.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/button.inc
