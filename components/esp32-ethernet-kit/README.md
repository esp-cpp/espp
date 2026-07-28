# ESP32-Ethernet-Kit A V1.2

[![Badge](https://components.espressif.com/components/espp/esp32-ethernet-kit/badge.svg)](https://components.espressif.com/components/espp/esp32-ethernet-kit)

Board Support Package (BSP) for the Espressif **ESP32-Ethernet-Kit A V1.2**.

## Official board documentation

- [ESP32-Ethernet-Kit overview](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-ethernet-kit/index.html)
- [ESP32-Ethernet-Kit V1.2 User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-ethernet-kit/user_guide_v1.2.html)
- [Board schematic V1.2](https://dl.espressif.com/dl/schematics/SCH_ESP32-ETHERNET-KIT_A_V1.2_20200528.pdf)

## What this BSP provides

The `espp::Esp32EthernetKit` class is a singleton hardware abstraction for:

- **10/100 Ethernet** — internal ESP32 EMAC + IP101GRI RMII PHY, selectable
  DHCP client **or** DHCP server mode

## Initialization API

The BSP exposes two initialization entry points:

- `initialize_ethernet()`
- `initialize_ethernet(const EthernetConfig &config)`

The no-argument overload is equivalent to `EthernetConfig{}` (DHCP client mode,
no callbacks).

```cpp
auto &board = espp::Esp32EthernetKit::get();
bool ok = board.initialize_ethernet();
```

## DHCP modes and callbacks

Mode selection is done through `EthernetConfig::mode`:

- `DhcpMode::CLIENT` (default): obtains an address from an upstream DHCP server.
- `DhcpMode::SERVER`: serves leases to connected hosts from a static interface IP.

Callback behavior:

- `on_link_up`: cable/link negotiated.
- `on_link_down`: link lost.
- `on_got_ip`: IPv4 becomes usable.
  - Client mode: after DHCP lease is obtained.
  - Server mode: when link comes up (static IP is already known).
- `on_lost_ip`: IPv4 no longer usable.
- `server_config.on_client_assigned` (server mode only): fires for each lease assignment.

### DHCP client (default)

```cpp
using Kit = espp::Esp32EthernetKit;

Kit::EthernetConfig cfg;
cfg.mode = Kit::DhcpMode::CLIENT;
cfg.on_link_up = []() {
  // physical link is up
};
cfg.on_got_ip = [](esp_ip4_addr_t ip) {
  // DHCP lease acquired
};
cfg.on_lost_ip = []() {
  // lease lost or interface disconnected
};

auto &board = Kit::get();
bool ok = board.initialize_ethernet(cfg);
```

### DHCP server

```cpp
using Kit = espp::Esp32EthernetKit;

Kit::EthernetConfig cfg;
cfg.mode = Kit::DhcpMode::SERVER;

// Leave ip_info all-zero to use default 192.168.4.1/24.
// Or set custom static address information:
// IP4_ADDR(&cfg.server_config.ip_info.ip, 10, 0, 0, 1);
// IP4_ADDR(&cfg.server_config.ip_info.netmask, 255, 255, 255, 0);
// IP4_ADDR(&cfg.server_config.ip_info.gw, 10, 0, 0, 1);

cfg.on_got_ip = [](esp_ip4_addr_t ip) {
  // server interface IP became active
};
cfg.server_config.on_client_assigned = [](esp_ip4_addr_t ip, std::array<uint8_t, 6> mac) {
  // a client received a lease
};

auto &board = Kit::get();
bool ok = board.initialize_ethernet(cfg);
```

## RMII pin mapping

The ESP32 RMII data-plane signals are fixed to specific GPIOs via IO_MUX and
**cannot be changed**. The control-plane signals (MDC/MDIO/PHY_RST) can be
routed via the GPIO matrix.

| Signal       | GPIO | Notes                                      |
|--------------|------|--------------------------------------------|
| REF_CLK (in) |    0 | External 50 MHz oscillator on V1.2         |
| TX_EN        |   21 | IO_MUX — fixed                             |
| TXD0         |   19 | IO_MUX — fixed                             |
| TXD1         |   22 | IO_MUX — fixed                             |
| CRS_DV       |   27 | IO_MUX — fixed                             |
| RXD0         |   25 | IO_MUX — fixed                             |
| RXD1         |   26 | IO_MUX — fixed                             |
| MDC          |   23 | GPIO matrix — reconfigurable               |
| MDIO         |   18 | GPIO matrix — reconfigurable               |
| PHY_RST      |    5 | Active-low; set `eth_phy_reset_gpio = -1` to skip |

> [!WARNING]
> **GPIO0 / REF_CLK conflict.** GPIO0 is both the RMII REF_CLK input (driven by
> the on-board 50 MHz oscillator) and the BOOT strapping pin. Pressing BOOT while
> Ethernet is running briefly pulls the clock line to GND, disrupting the 50 MHz
> clock and corrupting active traffic. Do **not** use GPIO0 as a runtime input
> while Ethernet is active.

## sdkconfig requirements

```
CONFIG_ETH_ENABLED=y
CONFIG_ETH_USE_ESP32_EMAC=y
CONFIG_ETH_PHY_ENABLE_IP101=y
```
