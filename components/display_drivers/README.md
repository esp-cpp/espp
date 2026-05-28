# Display Drivers Components

[![Badge](https://components.espressif.com/components/espp/display_drivers/badge.svg)](https://components.espressif.com/components/espp/display_drivers)

This component contains a set of reusable display-controller implementations
designed to be used with the `display` component.

The drivers now follow the shared object-style
`display_drivers::Controller` / `display_drivers::MipiDbiDisplayDriver`
architecture, with controller state owned per instance instead of hidden behind
static globals.

It also contains the `SpiPanelIo` transport helper in
`include/spi_panel_io.hpp` (also aliased as `SpiCommandData`) for SPI-connected
command/data panels built on top of the shared `spi` component.

## Included Drivers

- `Gc9a01`
- `Ili9341`
- `Ili9881`
- `Sh8601`
- `Ssd1351`
- `St7123`
- `St7789`
- `St7796`

## Example

The [example](./example) is designed to show how the `display_drivers` component
can be used to drive various different displays with LVGL and a simple GUI (that
is contained within the example: `main/gui.hpp`).
