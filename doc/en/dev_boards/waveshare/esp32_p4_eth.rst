Waveshare ESP32-P4-ETH
**********************

ESP32-P4-ETH
------------

The Waveshare ESP32-P4-ETH is an ESP32-P4 development board with 10/100
Ethernet (internal EMAC + IP101GRI RMII PHY).

The ``espp::Esp32P4Eth`` component provides a singleton hardware abstraction for
bringing up the board's Ethernet, delegating to the reusable :doc:`espp::Ethernet
<../../network/ethernet>` component and supplying the board-specific RMII pins.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_p4_eth_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-p4-eth.inc
