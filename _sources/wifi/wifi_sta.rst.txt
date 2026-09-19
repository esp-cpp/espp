WiFi Station (STA)
******************

The WiFi station enables the ESP to scan for and connect to an exising WiFi
access point.

There is an associated menu class `espp::WifiStaMenu` which provides methods to
configure the station settings via a menu interface at runtime.

You can use this class directly, or you can access / manage it via the singleton
`espp::Wifi` class.

.. ------------------------------- Example -------------------------------------

.. toctree::

   wifi_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/wifi_sta.inc
.. include-build-file:: inc/wifi_sta_menu.inc
