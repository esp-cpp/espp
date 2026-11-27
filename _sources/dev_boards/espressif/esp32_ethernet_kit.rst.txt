ESP32-Ethernet-Kit A V1.2
*************************

Esp32-Ethernet-Kit
------------------

The ESP32-Ethernet-Kit A V1.2 is an Espressif development board built around
the ESP32, featuring a wired 10/100 Ethernet port via the on-board IP101GRI
PHY connected to the ESP32's internal EMAC over RMII.

The `espp::Esp32EthernetKit` component provides a singleton hardware abstraction
for initializing the Ethernet interface in either **DHCP client** or **DHCP
server** mode.

**DHCP client** (default): the ESP32 requests an IP address from an upstream
router or switch. The ``on_link_up`` callback fires once the DHCP lease is
granted.

**DHCP server**: the ESP32 assigns IP addresses to connected hosts. The interface
uses a static IP (default ``192.168.4.1/24``, configurable via
:cpp:class:`espp::Esp32EthernetKit::ServerConfig`). The ``on_link_up`` callback
fires immediately when the cable is connected. An optional
``on_client_assigned`` callback fires each time a DHCP lease is issued to a
client.

.. warning::

   **GPIO0 / REF_CLK conflict.** GPIO0 is both the RMII REF_CLK input (driven
   by the on-board 50 MHz oscillator) and the ESP32 BOOT strapping pin.
   Pressing BOOT while Ethernet is active briefly pulls the clock line to GND,
   disrupting the 50 MHz clock and corrupting active traffic. Do **not** use
   GPIO0 as a runtime input while Ethernet is active.

RMII pin mapping (fixed via ESP32 IO_MUX; cannot be reassigned):

+-------------+------+------------------------------------------+
| Signal      | GPIO | Notes                                    |
+=============+======+==========================================+
| REF_CLK in  |    0 | External 50 MHz oscillator (V1.2)        |
+-------------+------+------------------------------------------+
| TX_EN       |   21 | IO_MUX — fixed                           |
+-------------+------+------------------------------------------+
| TXD0        |   19 | IO_MUX — fixed                           |
+-------------+------+------------------------------------------+
| TXD1        |   22 | IO_MUX — fixed                           |
+-------------+------+------------------------------------------+
| CRS_DV      |   27 | IO_MUX — fixed                           |
+-------------+------+------------------------------------------+
| RXD0        |   25 | IO_MUX — fixed                           |
+-------------+------+------------------------------------------+
| RXD1        |   26 | IO_MUX — fixed                           |
+-------------+------+------------------------------------------+
| MDC         |   23 | GPIO matrix — reconfigurable             |
+-------------+------+------------------------------------------+
| MDIO        |   18 | GPIO matrix — reconfigurable             |
+-------------+------+------------------------------------------+
| PHY_RST     |    5 | Active-low                               |
+-------------+------+------------------------------------------+

Official board documentation:

- `ESP32-Ethernet-Kit overview <https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-ethernet-kit/index.html>`_
- `ESP32-Ethernet-Kit V1.2 User Guide <https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-ethernet-kit/user_guide_v1.2.html>`_
- `Board schematic V1.2 (PDF) <https://dl.espressif.com/dl/schematics/SCH_ESP32-ETHERNET-KIT_A_V1.2_20200528.pdf>`_

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_ethernet_kit_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-ethernet-kit.inc
