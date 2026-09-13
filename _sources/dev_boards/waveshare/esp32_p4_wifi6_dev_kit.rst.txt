Waveshare ESP32-P4-WIFI6-DEV-KIT
********************************

ESP32-P4-WIFI6-DEV-KIT
----------------------

The Waveshare ESP32-P4-WIFI6-DEV-KIT is an ESP32-P4 development board with an
onboard ESP32-C6 co-processor (Wi-Fi 6 / Bluetooth 5 LE over SDIO via
ESP-Hosted), a MIPI-DSI display connector, capacitive touch, a MIPI-CSI camera
connector, audio in/out with a headphone jack, a microSD slot, 10/100 Ethernet
(with an optional external PoE-module header), and a Raspberry-Pi-compatible
40-pin GPIO header.

The ``espp::Esp32P4Wifi6DevKit`` component provides a singleton hardware
abstraction for bringing up the board's peripherals:

- **Display:** a MIPI-DSI panel (JD9365 or ILI9881C 10.1" 800x1280, or EK79007
  7" 1024x600, selected via Kconfig), with an LVGL display driver.
- **Touch:** a GT911 capacitive multi-touch controller (polled — the INT / RST
  lines are not routed to the ESP32-P4 on this board).
- **Camera:** a MIPI-CSI camera (OV5647 by default) captured through esp_video
  (V4L2), delivering RGB565 frames.
- **Audio:** an ES8311 codec with an NS4150B amplifier for speaker output, and
  the onboard analog microphone through the codec's ADC (full-duplex over I2S).
  Inserting headphones into the 3.5 mm jack mutes the speaker amplifier in
  hardware.
- **microSD / TF card:** a 4-bit SDMMC slot powered by the on-chip LDO.
- **Ethernet:** 10/100 via the internal EMAC and an IP101GRI RMII PHY,
  delegating to the reusable :doc:`espp::Ethernet <../../network/ethernet>`
  component and supplying the board-specific RMII pins (DHCP client / server).

The ES8311 codec, GT911 touch controller, and camera SCCB share a single
internal I2C bus. The camera pipeline requires PSRAM and the MIPI-CSI Kconfig
options enabled (see the example's ``sdkconfig.defaults``).

Wi-Fi / Bluetooth are not wrapped by the BSP: the onboard ESP32-C6 runs the
ESP-Hosted slave firmware and is used through the ``espressif/esp_hosted`` +
``espressif/esp_wifi_remote`` managed components, after which the standard
``esp_wifi`` API works unchanged (the board wiring matches the ESP-Hosted SDIO
defaults for the ESP32-P4). See the example README for the exact steps.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_p4_wifi6_dev_kit_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-p4-wifi6-dev-kit.inc
