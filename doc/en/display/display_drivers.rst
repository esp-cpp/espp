Display Drivers
***************

ESPP contains a few different display drivers in the `display_drivers`
component, corresponding to common display drivers on espressif development
boards.

The component also provides `SpiPanelIo` / `SpiCommandData`, a SPI-backed
implementation of the shared `display_drivers::PanelIo` transport interface for
command/data display panels, in ``components/display_drivers/include/spi_panel_io.hpp``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   display_drivers_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/display_drivers.inc
.. include-build-file:: inc/gc9a01.inc
.. include-build-file:: inc/ili9341.inc
.. include-build-file:: inc/sh8601.inc
.. include-build-file:: inc/spi_panel_io.inc
.. include-build-file:: inc/ssd1351.inc
.. include-build-file:: inc/st7789.inc
.. include-build-file:: inc/st7796.inc
