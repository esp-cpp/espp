WiFi
****

The WiFi APIs enable the ESP to connect to wireless networks in both
station and access point modes.

The `espp::Wifi` is a singleton class which provides methods to configure and
manage WiFi connections. You are able to register multiple different station and
access point configurations with unique names and switch between them at
runtime.

.. ------------------------------- Example -------------------------------------

.. toctree::

   wifi_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/wifi.inc
