# SPI Component

[![Badge](https://components.espressif.com/components/espp/spi/badge.svg)](https://components.espressif.com/components/espp/spi)

The `Spi` class provides a simple C++ wrapper around the ESP-IDF SPI master
driver. It manages SPI bus initialization, per-device registration, blocking
transfers, queued transactions, and bus acquisition.

## Example

The [example](./example) shows how to create an SPI bus, attach a device, and
perform a simple addressed read transaction.
