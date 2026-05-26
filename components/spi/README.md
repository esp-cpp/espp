# SPI Component

[![Badge](https://components.espressif.com/components/espp/spi/badge.svg)](https://components.espressif.com/components/espp/spi)

The `Spi` class provides a simple C++ wrapper around the ESP-IDF SPI master
driver. It manages SPI bus initialization, per-device registration, blocking
transfers, queued transactions, and bus acquisition.

The component also provides `SpiPanelIo` (also aliased as `SpiCommandData`), a
helper for the common LCD / display pattern where a dedicated D/C pin selects
between command and data transactions while the pixel data is queued using DMA.

`SpiPanelIo` treats command/data selection consistently:

- `write_command()` queues the command byte with D/C low and automatically sends
  any parameter bytes with D/C high.
- `queue_command()` queues a command transaction with D/C low.
- `queue_data()` and `queue_pixels()` queue payload transactions with D/C high.

That means controller code should call `queue_command()` for register opcodes
and `queue_data()` / `queue_pixels()` for the corresponding payload without
manually OR-ing the D/C flag.

## Example

The [example](./example) shows how to create an SPI bus, attach a device, and
perform a simple addressed read transaction.
