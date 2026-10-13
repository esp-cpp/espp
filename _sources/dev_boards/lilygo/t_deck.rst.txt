LilyGo T-Deck
*************

T-Deck
------

The LilyGo T-Deck is an ESP32-S3 development board with a touchscreen display,
keyboard, trackball, audio output, and expansion headers.

The `espp::TDeck` component provides a singleton hardware abstraction for
initializing the touch, display, keyboard, trackball, audio, and micro-SD card
subsystems. The LCD now uses the shared `espp::Spi` / `SpiPanelIo` transport
path while SDSPI-mounted microSD access stays on the same SPI host.

.. ------------------------------- Example -------------------------------------

.. toctree::

   t_deck_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/t-deck.inc
