# Ethernet Component

[![Badge](https://components.espressif.com/components/espp/ethernet/badge.svg)](https://components.espressif.com/components/espp/ethernet)

The `ethernet` component provides a single C++ class, `espp::Ethernet`, that
wraps the ESP-IDF `esp_eth` APIs and brings up an Ethernet interface over either
transport with a clean, uniform configuration:

- **RMII** — the internal EMAC on SoCs that have one (`SOC_EMAC_SUPPORTED`:
  ESP32, ESP32-P4) plus an external RMII PHY, using the generic 802.3 PHY driver
  (supports IP101, LAN87xx, DP83848, RTL8201, KSZ8041, …).
- **SPI** — an external MAC+PHY chip (WIZnet W5500, and gated support for DM9051
  / ENC28J60) on a caller-owned SPI bus. Works on any SoC.
- **Pre-built driver** — an escape hatch that takes a caller-created
  `esp_eth_mac_t*` / `esp_eth_phy_t*` for any other chip.

It owns all of the boilerplate that BSPs otherwise duplicate — netif, the event
loop, netif glue, DHCP client/server, static IP, hostname, MAC assignment
(explicit or eFuse-derived), link/IP event dispatch to `std::function`
callbacks — and, unlike the inline BSP implementations it replaces, provides a
symmetric teardown (`deinitialize()` + destructor).

## Interface selection

The interface-specific configuration is a single tagged `Config::interface`
member (`std::variant`):

- `Ethernet::RmiiConfig{ .mdc_gpio, .mdio_gpio, .phy_addr, .phy_reset_gpio, ... }`
- `Ethernet::SpiConfig{ .host, .cs_gpio, .int_gpio, .reset_gpio, .chip }`
- `Ethernet::DriverConfig{ .mac, .phy }`

The rest of `Config` (DHCP `mode`, static `ip_info`, `hostname`, `mac_address`,
and the `on_link_up` / `on_link_down` / `on_got_ip` / `on_lost_ip` /
`on_client_assigned` callbacks) is shared across all interfaces.

## SPI chip drivers

The concrete SPI chip drivers are ESP-IDF managed components that are pulled in
only when the matching Kconfig option is enabled, so an RMII-only project does
not carry them:

- `CONFIG_ESPP_ETHERNET_W5500` → `espressif/w5500`
- `CONFIG_ESPP_ETHERNET_DM9051`
- `CONFIG_ESPP_ETHERNET_ENC28J60`

## Lifecycle

```cpp
espp::Ethernet eth(config);   // constructs (does not start)
eth.initialize(ec);           // create + attach + start (idempotent)
// ... eth.is_connected(), eth.get_ip_address(), eth.link_speed_duplex() ...
eth.deinitialize();           // reverse-order teardown (also called by ~Ethernet)
```

Each fallible method has both a `bool f(std::error_code& ec)` form and a
logging convenience overload.

## Example

The [example](./example) brings up a DHCP-client interface: RMII by default (on
an EMAC-capable target such as the ESP32-Ethernet-Kit), or a W5500 over SPI when
`CONFIG_ESPP_ETHERNET_W5500` is enabled.
