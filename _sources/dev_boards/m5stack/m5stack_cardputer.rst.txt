M5Stack Cardputer
*****************

M5Stack-Cardputer
-----------------

The M5Stack Cardputer (K132) and Cardputer ADV are card-sized computers based
on the ESP32-S3 StampS3 module. They feature a 56-key QWERTY keyboard, a 1.14"
240x135 IPS display, a mono speaker, a microphone, a micro-SD card slot, an IR
transmitter, a Grove port, and a WS2812 RGB LED.

The `espp::M5StackCardputer` component supports both variants with the same
API - the board is detected at runtime - and provides a singleton hardware
abstraction for initializing the display, keyboard (74HC138 GPIO matrix on the
original, TCA8418 I2C controller on the ADV), audio (NS4168 amplifier on the
original, ES8311 codec on the ADV), microphone, uSD card, RGB LED, battery
measurement, and button subsystems.

.. note::

   The speaker and the microphone share I2S pins, so they cannot be used at
   the same time; initializing one while the other is active will fail.

.. ------------------------------- Example -------------------------------------

.. toctree::

   m5stack_cardputer_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/m5stack-cardputer.inc
