# ESP32-P4-NANO Board Support Package (BSP)

[![Badge](https://components.espressif.com/components/espp/esp32-p4-nano/badge.svg)](https://components.espressif.com/components/espp/esp32-p4-nano)

Board Support Package for the [Waveshare ESP32-P4-NANO](https://www.waveshare.com/wiki/ESP32-P4-NANO)
board, exposing its 10/100 Ethernet (ESP32-P4 internal EMAC + IP101GRI RMII PHY).

The Ethernet bring-up is delegated to the reusable `espp::Ethernet` component;
this BSP just supplies the board-specific RMII pin mapping. It supports DHCP
client and DHCP-server modes with link/IP callbacks.

## Example

The [example](./example) brings up a DHCP-client interface and logs link / IP
state.
