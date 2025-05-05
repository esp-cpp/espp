# Seeed Studio Round Display Board Support Package (BSP) Component

The Seeed Studio Round Display is a development board containing a 240x240 round
display with touchscreen, RTC, battery charger, and uSD card slot. It support
connection to a QtyPy or a XIAO module.

The `espp::SsRoundDisplay` component provides a singleton hardware abstraction
for initializing the display and touchscreen components for the Seeed Studio
Round Display.

## Example

This example shows how to use the `espp::SsRoundDisplay` hardware abstraction
component to initialize the hardware components on the [Seeed Studio Round
Display](https://wiki.seeedstudio.com/get_start_round_display/) when used with
either the XIAO S3 or the QtPy ESP32S3.
