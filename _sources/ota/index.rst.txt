Firmware Update (OTA) APIs
**************************

.. toctree::
    :maxdepth: 1

    ota

The `Ota` component provides a transport-agnostic OTA firmware update engine:
stream a new app image into it from any transport (USB vendor / WebUSB, HTTP
over WiFi or Ethernet, sockets, UART, ...) and it validates, activates and
optionally rolls back the update.
