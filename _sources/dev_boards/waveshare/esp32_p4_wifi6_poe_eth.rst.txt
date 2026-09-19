Waveshare ESP32-P4-WIFI6-POE-ETH
********************************

ESP32-P4-WIFI6-POE-ETH
----------------------

The Waveshare ESP32-P4-WIFI6-POE-ETH pairs an ESP32-P4 (dual-core 360 MHz
RISC-V, 32 MB stacked PSRAM, 32 MB NOR flash) with an on-board ESP32-C6-MINI-1
Wi-Fi 6 / Bluetooth 5 LE co-processor and a 10/100 RJ45 Ethernet port with a
reserved header for Waveshare's plug-in PoE power module.

The ``espp::Esp32P4Wifi6PoeEth`` component provides a singleton hardware
abstraction for bringing up the board's peripherals:

- **Ethernet:** 10/100 via the internal EMAC and an IP101GRI RMII PHY,
  delegating to the reusable :doc:`espp::Ethernet <../../network/ethernet>`
  component and supplying the board-specific RMII pins (DHCP client / server).
  PoE is purely a power-supply feature — no GPIO is involved, so a PoE-powered
  board looks identical to a USB-powered one in software.
- **Internal I2C bus:** shared by the on-board ES8311 audio codec and the
  MIPI-DSI / MIPI-CSI connectors and 40-pin header.

.. note::
   The board's BOOT key shares GPIO35 with the RMII TXD1 line (GPIO35 is the
   ESP32-P4's boot-strap pin, sampled only at reset), so it cannot be used as
   a runtime input and the BSP does not expose a button API.

Wi-Fi 6 / BLE are provided by the on-board ESP32-C6 over SDIO using
Espressif's ESP-Hosted + ``esp_wifi_remote`` components; the BSP documents the
board's SDIO wiring (which matches the ESP-Hosted defaults for the ESP32-P4)
rather than wrapping it — see the example README for how to enable Wi-Fi in
your own application.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_p4_wifi6_poe_eth_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-p4-wifi6-poe-eth.inc
