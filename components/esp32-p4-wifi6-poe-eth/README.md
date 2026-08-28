# ESP32-P4-WIFI6-POE-ETH

[![Badge](https://components.espressif.com/components/espp/esp32-p4-wifi6-poe-eth/badge.svg)](https://components.espressif.com/components/espp/esp32-p4-wifi6-poe-eth)

Board Support Package (BSP) for the Waveshare **ESP32-P4-WIFI6-POE-ETH**: an
ESP32-P4 (dual-core 360 MHz RISC-V, 32 MB stacked PSRAM, 32 MB NOR flash) paired
with an on-board ESP32-C6-MINI-1 Wi-Fi 6 / BLE co-processor (SDIO, ESP-Hosted)
and a 10/100 RJ45 Ethernet port with a reserved plug-in PoE-module header.

## Official board documentation

- [Waveshare wiki](https://www.waveshare.com/wiki/ESP32-P4-WIFI6-POE-ETH)
- [Waveshare docs platform](https://docs.waveshare.com/ESP32-P4-WIFI6-POE-ETH)
- [Board schematic](https://files.waveshare.com/wiki/ESP32-P4-WIFI6-POE-ETH/ESP32-P4-WIFI6-POE-ETH-Schematic.pdf)
- [Product page](https://www.waveshare.com/esp32-p4-wifi6-poe-eth.htm)

## What this BSP provides

The `espp::Esp32P4Wifi6PoeEth` class is a singleton hardware abstraction for:

- **10/100 Ethernet** — internal ESP32-P4 EMAC + IP101GRI RMII PHY, selectable
  DHCP client **or** DHCP server mode (delegates to the reusable
  `espp::Ethernet` component).
- **BOOT button** — GPIO35 (active low), interrupt-driven with press/release
  callbacks.
- **Internal I2C bus** — SDA=GPIO7 / SCL=GPIO8, shared by the on-board ES8311
  audio codec (address 0x18) and the DSI / CSI connectors and 40-pin header.

PoE is purely a power-supply feature: the RJ45 center taps feed two MB10F
bridge rectifiers and a 5-pin header for Waveshare's plug-in PoE module, which
produces the board's 5 V rail. No GPIO is involved — a PoE-powered board is
indistinguishable from a USB-powered one in software.

## Wi-Fi 6 / Bluetooth LE (ESP32-C6 over SDIO, ESP-Hosted)

Wireless connectivity is provided by the on-board ESP32-C6 running Espressif's
ESP-Hosted slave firmware, connected to the P4 over SDIO. espp does not wrap
ESP-Hosted in a BSP API (yet); use Espressif's
[`esp_hosted`](https://components.espressif.com/components/espressif/esp_hosted)
and [`esp_wifi_remote`](https://components.espressif.com/components/espressif/esp_wifi_remote)
managed components directly — see the [example README](example/README.md) for
the exact steps and Kconfig pin settings. The wiring (from the schematic):

| Function            | ESP32-P4 GPIO | ESP32-C6 pin      |
|---------------------|---------------|-------------------|
| SDIO CLK            | 18            | IO19 (SDIO_CLK)   |
| SDIO CMD            | 19            | IO18 (SDIO_CMD)   |
| SDIO D0             | 14            | IO20 (SDIO_DATA0) |
| SDIO D1             | 15            | IO21 (SDIO_DATA1) |
| SDIO D2             | 16            | IO22 (SDIO_DATA2) |
| SDIO D3             | 17            | IO23 (SDIO_DATA3) |
| C6 reset (CHIP_PU)  | 54            | EN                |
| Spare (0 Ω link)    | 6             | IO2               |

This matches the default ESP-Hosted SDIO pin assignment for the ESP32-P4 (the
same wiring as Espressif's ESP32-P4-Function-EV-Board). A 4-pin header (H7)
exposes the C6's UART (U0TXD / U0RXD / IO9) for (re)flashing the C6 slave
firmware.

## RMII pin mapping

Identical to the Waveshare ESP32-P4-ETH / ESP32-P4-NANO. The IP101GRI generates
the 50 MHz REF_CLK from its own 25 MHz crystal; the P4 receives it on GPIO50.

| Signal       | GPIO | Notes                                  |
|--------------|------|----------------------------------------|
| REF_CLK (in) |   50 | 50 MHz from the PHY                    |
| TX_EN        |   49 |                                        |
| TXD0         |   34 |                                        |
| TXD1         |   35 |                                        |
| CRS_DV       |   28 |                                        |
| RXD0         |   29 |                                        |
| RXD1         |   30 |                                        |
| MDC          |   31 |                                        |
| MDIO         |   52 |                                        |
| PHY_RST      |   51 | Active low                             |

PHY address: 1 (PHY_AD0 strapped high via 5.1 kΩ pull-up).

## Other on-board hardware (not wrapped by this BSP)

Pin data from the schematic, for future expansion / user code:

- **Audio**: ES8311 codec (I2C 0x18) + NS4150B amplifier + analog microphone.
  I2S: MCLK=13, BCLK=12, WS/LRCK=10, DSDIN=9 (P4→codec), ASDOUT=11 (codec→P4);
  PA enable=GPIO53. Speaker on an MX1.25 2-pin connector.
- **microSD / TF**: 4-bit SDMMC on the fixed IO-MUX pins CLK=43, CMD=44,
  D0..D3=39/40/41/42; line pull-ups powered from the P4's on-chip LDO_VO4.
- **MIPI-DSI / MIPI-CSI**: 15-pin Raspberry-Pi-style FPC connectors (display /
  camera sold separately; included in the Kit-C / Kit-D bundles).
- **USB**: stacked USB-A (native USB 2.0 OTG HS) over USB-C (CH343P USB-UART
  console on UART0: TX=GPIO37, RX=GPIO38, with auto reset/boot).
- **40-pin header**: 28 free GPIOs, 3V3 / 5 V rails.
- **LEDs**: the red LED is a 5 V power indicator; the RJ45 green/yellow LEDs
  are driven by the IP101GRI. None are GPIO-controllable.

## Example

The [example](./example) initializes Ethernet (DHCP client or server via
`menuconfig`), prints the acquired IP address periodically, and hooks up the
BOOT button.

## sdkconfig requirements

```
CONFIG_ETH_ENABLED=y
CONFIG_ETH_USE_ESP32_EMAC=y
```
