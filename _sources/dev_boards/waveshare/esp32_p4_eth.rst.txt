Waveshare ESP32-P4-ETH
**********************

ESP32-P4-ETH
------------

The Waveshare ESP32-P4-ETH is an ESP32-P4 development board with a MIPI-DSI
display, capacitive touch, a MIPI-CSI camera, audio in/out, a microSD slot, and
10/100 Ethernet.

The ``espp::Esp32P4Eth`` component provides a singleton hardware abstraction for
bringing up the board's peripherals:

- **Display:** a MIPI-DSI panel (ILI9881C 10.1" 800x1280 or EK79007 7"
  1024x600, selected via Kconfig), with an LVGL display driver.
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

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_p4_eth_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-p4-eth.inc
