Ethernet
********

The :cpp:class:`espp::Ethernet` component wraps the ESP-IDF ``esp_eth`` APIs and
brings up an Ethernet interface over either **RMII** (internal EMAC, on SoCs with
``SOC_EMAC_SUPPORTED``) or **SPI** (an external MAC+PHY chip such as the WIZnet
W5500), plus a pre-built-driver escape hatch for any other chip.

One class owns the boilerplate that BSPs otherwise duplicate - netif, event loop,
netif glue, DHCP client/server, static IP, hostname, MAC assignment (explicit or
eFuse-derived) and link/IP event dispatch to ``std::function`` callbacks - and,
unlike the inline BSP implementations it replaces, provides a symmetric teardown
(``deinitialize()`` + destructor).

The interface-specific configuration is a single tagged ``Config::interface``
member (a ``std::variant`` of ``RmiiConfig`` / ``SpiConfig`` / ``DriverConfig``);
the rest of ``Config`` (DHCP mode, static IP, hostname, MAC, callbacks) is shared.

.. note::

   The RMII path uses the generic 802.3 PHY driver in ``esp_eth`` core (no extra
   dependency). The concrete SPI chip drivers (W5500, DM9051, ENC28J60) are
   managed components pulled in only when the matching ``CONFIG_ESPP_ETHERNET_*``
   Kconfig option is enabled.

.. ------------------------------- Example -------------------------------------

.. toctree::

   ethernet_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/ethernet.inc
