LilyGo T5 4.7" e-paper
**********************

T5 4.7"
-------

The LilyGo T5 4.7" (ESP32-S3) is an e-paper development board built around a
4.7" 960x540, 16-level-grayscale ED047TC1 panel driven over a parallel bus (the
ESP32-S3 ``LCD_CAM`` peripheral). This BSP wraps the
`epdiy <https://github.com/vroland/epdiy>`_ library, which owns the panel
timing, waveforms, grayscale rendering, and partial-update handling.

The `espp::LilyGoT547` component provides a singleton hardware abstraction that
brings up the e-paper display, an LVGL grayscale (``L8``) display, the GT911
capacitive touch (as an LVGL input device), the PCF8563 RTC, the BQ27220 battery
fuel gauge, the PCA9535 I/O expander (including the IO48 button), the BOOT
button, the SX1262 LoRa radio, the frontlight, the qwiic connector, and the
microSD card.

epdiy's e-paper power ICs and the board's other I2C peripherals share a single
internal I2C bus (SDA=39 / SCL=40) that this BSP creates and hands to epdiy, so
touch, RTC, battery gauge, I/O expander, and qwiic all live on the same bus.

.. ------------------------------- Example -------------------------------------

.. toctree::

   lilygo_t5_47_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/lilygo-t5-47.inc
