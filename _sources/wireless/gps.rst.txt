GPS / GNSS
**********

The `Gps` component provides a driver for UART-attached GNSS receivers which
output standard NMEA-0183 sentences (e.g. the ATGM336H on the M5Stack
Cardputer LoRa+GPS Cap, or the u-blox MIA-M10Q on the LilyGo T-Deck Plus). It
reads and parses the NMEA stream in a background task and provides the latest
fix (position, altitude, speed, course, satellites, UTC time) thread-safely or
via callback.

The `NmeaParser` class provides the underlying platform-independent NMEA
parsing and can be used on its own.

.. toctree::

   gps_example

API Reference
-------------

.. include-build-file:: inc/gps.inc
.. include-build-file:: inc/nmea_parser.inc
.. include-build-file:: inc/gps_fix.inc
