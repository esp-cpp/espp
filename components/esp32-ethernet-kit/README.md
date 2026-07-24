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

## DHCP modes

`initialize_ethernet` accepts a `DhcpMode` parameter (default `CLIENT`).

### DHCP client (default)

The ESP32 requests an IP address from an upstream router/switch.
The `on_link_up` callback fires once the DHCP lease is granted.

```cpp
auto &board = espp::Esp32EthernetKit::get();

board.initialize_ethernet(
    [](esp_ip4_addr_t ip) {
      // called when lease is acquired
    },
    espp::Esp32EthernetKit::DhcpMode::CLIENT); // default — can be omitted
```

### DHCP server

The ESP32 assigns IP addresses to connected hosts using a static IP.
The `on_link_up` callback fires when the cable is connected (IP is static, so
it is known immediately). An optional `on_client_assigned` callback fires each
time a client receives a lease.

```cpp
using DhcpMode    = espp::Esp32EthernetKit::DhcpMode;
using ServerConfig = espp::Esp32EthernetKit::ServerConfig;

auto &board = espp::Esp32EthernetKit::get();

ServerConfig cfg;
// Leave cfg.ip_info zero to use the built-in default: 192.168.4.1 / 255.255.255.0
// Or set a custom address:
//   IP4_ADDR(&cfg.ip_info.ip,      10, 0, 0, 1);
//   IP4_ADDR(&cfg.ip_info.netmask, 255, 255, 255, 0);
//   IP4_ADDR(&cfg.ip_info.gw,      10, 0, 0, 1);

cfg.on_client_assigned = [](esp_ip4_addr_t ip, std::array<uint8_t, 6> mac) {
  // fired each time a client gets a DHCP lease
};

board.initialize_ethernet(
    [](esp_ip4_addr_t ip) {
      // called when link is up (static IP is already known)
    },
    DhcpMode::SERVER,
    cfg);
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
