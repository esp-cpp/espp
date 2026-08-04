# Ethernet Example

This example demonstrates the `espp::Ethernet` component bringing up a
DHCP-client interface and logging link / IP state.

By default it uses **RMII** (internal EMAC + an RMII PHY) on an EMAC-capable
target such as the ESP32-Ethernet-Kit. Enable `CONFIG_ESPP_ETHERNET_W5500` to
build the **SPI / W5500** variant instead (which also pulls in the
`espressif/w5500` managed component).

Adjust the pins in `main/ethernet_example.cpp` for your board.

## How to use

```
idf.py set-target esp32
idf.py build flash monitor
```
