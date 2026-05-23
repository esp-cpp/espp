ST7123 Touch Controller
***********************

The `St7123Touch` class provides an interface to the capacitive touch controller
integrated in the **Sitronix ST7123** TDDI (Touch and Display Driver Integration)
chip.

The ST7123 combines a MIPI-DSI display driver and a multi-touch capacitive
controller in a single IC. This driver accesses the touch side over I2C
(default address **0x55**).

.. note::

   The ST7123's touch engine is enabled by the **LCD_RST** pulse issued during
   display initialization — do *not* toggle the ``TP_RST`` line used by
   standalone controllers such as the GT911, as this can take the touch I2C
   endpoint offline.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/st7123touch.inc
