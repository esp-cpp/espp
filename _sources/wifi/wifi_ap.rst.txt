WiFi Access Point (AP)
**********************

The WiFi access point enables the ESP to host its own WiFi network to which
other devices can connect.

There is an associated menu class `espp::WifiApMenu` which provides methods to
configure the access point settings via a menu interface at runtime.

You can use this class directly, or you can access / manage it via the singleton
`espp::Wifi` class.

.. ------------------------------- Example -------------------------------------

.. toctree::

   wifi_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/wifi_ap.inc
.. include-build-file:: inc/wifi_ap_menu.inc
