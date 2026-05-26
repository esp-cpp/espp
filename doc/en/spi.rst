SPI
***

The `spi` component provides a C++ wrapper around ESP-IDF's SPI master driver.

`Spi` owns the bus, `Spi::Device` owns attached peripherals on that bus, and
`SpiCommandData` provides a higher-level helper for LCD-style command/data
transports that queue transactions and toggle a D/C GPIO in the SPI callbacks.

.. toctree::

   spi_example

API Reference
-------------

.. include-build-file:: inc/spi.inc
