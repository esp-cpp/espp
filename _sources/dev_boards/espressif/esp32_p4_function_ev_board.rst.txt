ESP32-P4 Function EV Board
**************************

ESP32-P4-Function-EV-Board
--------------------------

The ESP32-P4 Function EV Board is an Espressif development board for the ESP32-P4
microprocessor. Together with the ESP32-P4-HMI-Subboard it provides a MIPI-DSI
touchscreen display, an ES8311 audio codec with speaker amplifier, 10/100
Ethernet, a MIPI-CSI camera interface, a microSD card slot, and USB.

The `espp::Esp32P4FunctionEvBoard` component provides a singleton hardware
abstraction for initializing the display, touch, audio, Ethernet, and SD card
subsystems. The display panel (EK79007 1024x600 or ILI9881C 800x1280) is
selectable via Kconfig.

Touch is **polled** by default (the GT911 INT pin is not routed to the ESP32-P4
on this board). If you wire the INT pin from the LCD expansion header to a free
GPIO, you can switch to **interrupt-driven** touch via Kconfig
(``ESP_P4_EV_BOARD_TOUCH_INTERRUPT`` / ``..._GPIO``) or by passing the GPIO to
``initialize_touch()``.

Official board documentation:

- `ESP32-P4-Function-EV-Board User Guide <https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide.html>`_ (includes the ESP32-P4-HMI-Subboard)
- `ESP32-P4-Function-EV-Board overview page <https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/index.html>`_
- `Board schematic (PDF) <https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-schematics_v1.52.pdf>`_
- `Espressif reference BSP (esp-bsp) <https://github.com/espressif/esp-bsp/tree/master/bsp/esp32_p4_function_ev_board>`_
- `ESP32-P4 Get Started (ESP-IDF) <https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/get-started/index.html>`_

Display panels (HMI subboard). Both panels plug into the shared LCD adapter
board via its FPC connector:

- `LCD Adapter Board Schematic (PDF) <https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-lcd-subboard-schematics.pdf>`_
- `LCD Adapter Board PCB Layout (PDF) <https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-lcd-subboard-pcb-layout.pdf>`_

EK79007 (7", 1024x600) — the panel Espressif ships/documents for this board:

- `Display Datasheet (PDF) <https://dl.espressif.com/dl/schematics/display_datasheet.pdf>`_
- `EK79007AD display driver chip datasheet (PDF) <https://dl.espressif.com/dl/schematics/display_driver_chip_EK79007AD_datasheet.pdf>`_
- `EK73217BCGA display driver chip datasheet (PDF) <https://dl.espressif.com/dl/schematics/display_driver_chip_EK73217BCGA_datasheet.pdf>`_

ILI9881C (10.1", 800x1280) — a panel option supported by the BSP. Espressif does
not publish a dedicated LCD-subboard schematic/datasheet for this panel on this
board; refer to the `esp-bsp esp_lcd_ili9881c driver
<https://github.com/espressif/esp-bsp/tree/master/components/lcd/esp_lcd_ili9881c>`_
for the panel/timing details.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_p4_function_ev_board_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-p4-function-ev-board.inc
