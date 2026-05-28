# Display Drivers Components

[![Badge](https://components.espressif.com/components/espp/display_drivers/badge.svg)](https://components.espressif.com/components/espp/display_drivers)

This component contains a few different display drivers, corresponding to common
display drivers on espressif development boards. These display drivers are
designed to be used with the `display` component.

It also contains the `SpiPanelIo` transport helper in
`include/spi_panel_io.hpp` (also aliased as `SpiCommandData`) for SPI-connected
command/data panels built on top of the shared `spi` component.

## Example

The [example](./example) is designed to show how the `display_drivers` component
can be used to drive various different displays with LVGL and a simple GUI (that
is contained within the example: `main/gui.hpp`).
