ESP BOX
*******

Esp-Box
-------

The ESP32-S3-BOX and ESP32-S3-BOX-3 are development boards for the ESP32-S3
module. They feature a nice touchscreen display, a speaker, microphones, and
expansion headers.

The `espp::EspBox` component provides a singleton hardware abstraction for
initializing the touch, display, and audio subsystems, as well as automatically
determining which version of the Box it's running on.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp_box_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp-box.inc
