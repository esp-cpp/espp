# ESP32-P4 Function EV Board

[![Badge](https://components.espressif.com/components/espp/esp32-p4-function-ev-board/badge.svg)](https://components.espressif.com/components/espp/esp32-p4-function-ev-board)

Board Support Package (BSP) for the Espressif **ESP32-P4 Function EV Board** used
together with the **ESP32-P4-HMI-Subboard**.

> [!IMPORTANT]
> **Display jumpers.** On the LCD adapter board, the `RST_LCD` and `PWM` signals
> are broken out on a header and must be wired to the ESP32-P4 main board for the
> display to work: connect **`RST_LCD` → J1 GPIO27** and **`PWM` → J1 GPIO26**
> (these are the LCD reset and backlight-PWM pins this BSP drives). Without these
> jumpers the panel never gets reset and the backlight PWM is unconnected, so the
> screen stays black even though everything initializes. See the
> [user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide.html)
> for the header pinout.

## Official board documentation

- [ESP32-P4-Function-EV-Board User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide.html)
  (includes the ESP32-P4-HMI-Subboard)
- [ESP32-P4-Function-EV-Board overview page](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/index.html)
- [Board schematic (PDF)](https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-schematics_v1.52.pdf)
- [Espressif reference BSP (`esp-bsp`)](https://github.com/espressif/esp-bsp/tree/master/bsp/esp32_p4_function_ev_board)
- [ESP32-P4 Get Started (ESP-IDF)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/get-started/index.html)

### Display panels (HMI subboard)

Both panels plug into the shared LCD adapter board via its FPC connector:

- [LCD Adapter Board Schematic (PDF)](https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-lcd-subboard-schematics.pdf)
- [LCD Adapter Board PCB Layout (PDF)](https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-lcd-subboard-pcb-layout.pdf)

**EK79007 — 7", 1024x600** (the panel Espressif ships/documents for this board):

- [Display Datasheet (PDF)](https://dl.espressif.com/dl/schematics/display_datasheet.pdf)
- [EK79007AD display driver chip datasheet (PDF)](https://dl.espressif.com/dl/schematics/display_driver_chip_EK79007AD_datasheet.pdf)
- [EK73217BCGA display driver chip datasheet (PDF)](https://dl.espressif.com/dl/schematics/display_driver_chip_EK73217BCGA_datasheet.pdf)

**ILI9881C — 10.1", 800x1280**: a panel option supported by the BSP. Espressif
does not publish a dedicated LCD-subboard schematic/datasheet for this panel on
this board; refer to the
[esp-bsp `esp_lcd_ili9881c` driver](https://github.com/espressif/esp-bsp/tree/master/components/lcd/esp_lcd_ili9881c)
for the panel/timing details.

The `espp::Esp32P4FunctionEvBoard` class is a singleton hardware abstraction that
initializes and exposes the board's peripherals:

- **MIPI-DSI display** — Kconfig-selectable **EK79007 (7", 1024x600)** or
  **ILI9881C (10.1", 800x1280)** — with LVGL integration and PWM backlight.
- **GT911 capacitive multi-touch** (the touch interrupt is not routed on this
  board, so touch is polled).
- **ES8311 audio codec** (+ NS4150B speaker amplifier) over I2S for playback.
- **10/100 Ethernet** (EMAC + IP101 RMII PHY) with DHCP.
- **microSD card** (4-bit SDMMC, powered via the on-chip LDO).
- **MIPI-CSI camera** (SC2336/OV5647) — pins/SCCB wired, capture pipeline is a
  stub (see Camera below).
- **BOOT button**.

All on-board control lines are direct ESP32-P4 GPIOs (this board has no I/O
expander). The display, touch, and audio codec share a single I2C bus
(`SDA=GPIO7`, `SCL=GPIO8`).

## Display panel detection

`initialize_lcd()` **probes the attached panel at runtime**: it brings up the
MIPI-DSI bus, reads the ILI9881C ID over DSI, and selects **ILI9881C** if it
matches or **EK79007** otherwise. It then applies that panel's resolution, DPI
timing, backlight GPIO (26 for EK79007, 23 for ILI9881C), and reset GPIO, and
`get_display_controller()` / `display_width()` / `display_height()` reflect the
detected panel. The probed ID bytes are logged (`Panel probe ID: ...`).

The Kconfig choice under *ESP32-P4 Function EV Board Configuration* is only the
**fallback** used if probing is inconclusive.

## Example

See the [example](./example). It initializes the display + touch, shows a live
on-screen status read-out (panel, touch coordinates, SD, Ethernet IP), and brings
up the SD card, audio, Ethernet, and BOOT button.

## Peripheral status / notes

- **Ethernet**: uses the ESP-IDF internal EMAC with the generic 802.3 PHY driver
  for the IP101 (the dedicated `esp_eth_phy_ip101` driver is a separate managed
  component in newer ESP-IDF; the generic driver works for this board). The RMII
  pinout is the ESP-IDF ESP32-P4 default (MDC=31, MDIO=52, REF_CLK in=50,
  TX_EN=49, TXD0=34, TXD1=35, CRS_DV=28, RXD0=29, RXD1=30; PHY reset=51, addr=1).
- **Camera**: the SC2336/OV5647 MIPI-CSI sensor's pins are documented (SCCB on
  the internal I2C bus at 0x30, reset/XCLK not connected), but the `esp_video`
  capture pipeline is **not yet implemented**; `initialize_camera()` returns
  false. Contributions welcome.
- This BSP requires PSRAM for the MIPI-DSI frame buffers (see the example's
  `sdkconfig.defaults`).
- The Ethernet RMII pinout comes from the ESP-IDF ESP32-P4 software defaults
  (corroborated by the board docs) rather than a parsed schematic — verify
  against your board revision if Ethernet does not link.
