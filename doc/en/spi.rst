SPI
***

The `spi` component provides a C++ wrapper around ESP-IDF's SPI master driver.

`Spi` owns the bus and `Spi::Device` owns attached peripherals on that bus.
Display-specific command/data helpers such as `SpiPanelIo` / `SpiCommandData`
now live in the `display_drivers` component.

.. toctree::

   spi_example

API Reference
-------------

.. include-build-file:: inc/spi.inc
