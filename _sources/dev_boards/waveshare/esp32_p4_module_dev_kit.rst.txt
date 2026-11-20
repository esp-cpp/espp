Waveshare ESP32-P4-Module-DEV-KIT
*********************************

ESP32-P4-Module-DEV-KIT
-----------------------

The Waveshare ESP32-P4-Module-DEV-KIT is the Waveshare ESP32-P4-Module (ESP32-P4
with 16 MB flash, 32 MB PSRAM, and an on-module ESP32-C6 for WiFi 6 /
Bluetooth 5) on a carrier board with a MIPI-DSI display connector, capacitive
touch, a MIPI-CSI camera connector, audio in/out, a TF-card slot, USB 2.0 OTG
HS, and 10/100 Ethernet with an optional PoE module.

The ``espp::Esp32P4ModuleDevKit`` component provides a singleton hardware
abstraction for bringing up the board's peripherals:

- **Display:** a MIPI-DSI panel (JD9365 10.1" 800x1280 by default, or ILI9881C
  10.1" 800x1280 / EK79007 7" 1024x600, selected via Kconfig), with an LVGL
  display driver. The board is sold without a panel (the -C kit bundles the
  10.1" JD9365).
- **Touch:** a GT911 capacitive multi-touch controller (polled — the INT / RST
  lines are not routed to the ESP32-P4 on this board).
- **Camera:** a MIPI-CSI camera (OV5647 by default) captured through esp_video
  (V4L2), delivering RGB565 frames.
- **Audio:** an ES8311 codec with an NS4150B amplifier for speaker output, and
  the onboard analog microphone through the codec's ADC (full-duplex over I2S).
- **microSD / TF card:** a 4-bit SDMMC slot powered by the on-chip LDO.
- **Ethernet:** 10/100 via the internal EMAC and an IP101GRI RMII PHY,
  delegating to the reusable :doc:`espp::Ethernet <../../network/ethernet>`
  component and supplying the board-specific RMII pins (DHCP client / server).

The ES8311 codec, GT911 touch controller, and camera SCCB share a single
internal I2C bus. The camera pipeline requires PSRAM and the MIPI-CSI Kconfig
options enabled (see the example's ``sdkconfig.defaults``).

WiFi / Bluetooth are provided by the ESP32-C6 on the module (SDIO to the P4,
esp_hosted default pins) and are not managed by this BSP — use the
``esp_wifi_remote`` + ``esp_hosted`` components directly.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_p4_module_dev_kit_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-p4-module-dev-kit.inc
