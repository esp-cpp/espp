# Vendored third-party sources

## esp_eth_phy_ip101 (IP101 Ethernet PHY driver)

`esp_eth_phy_ip101.c` / `esp_eth_phy_ip101.h` are vendored verbatim from
Espressif's [`esp-eth-drivers`](https://github.com/espressif/esp-eth-drivers)
(`ip101/` component), licensed **Apache-2.0** (see `LICENSE`).

They are bundled here because, on ESP-IDF v6+, the vendor-specific Ethernet PHY
drivers (IP101, etc.) were moved out of the built-in `esp_eth` component into
separate registry components. The ESP32-P4 Function EV Board uses an **IP101**
PHY, and this BSP builds with the ESP-IDF component manager disabled (it consumes
the local `espp/*` components directly), so the driver is vendored rather than
fetched. It depends only on `esp_eth_phy_802_3.h`, which is part of the IDF's
`esp_eth` component.

To update: replace the two files from the upstream `ip101/` component (keeping
their SPDX license headers) and update this note.
