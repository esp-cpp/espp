# ESP32-P4 Function EV Board Example

This example demonstrates the `espp::Esp32P4FunctionEvBoard` BSP for the
Espressif ESP32-P4 Function EV Board + ESP32-P4-HMI-Subboard.

It:

- initializes the MIPI-DSI display (EK79007 or ILI9881C, per Kconfig) and the
  GT911 touch controller,
- draws an LVGL UI that lets you **draw circles wherever you touch** and plays a
  **click sound** on each touch (press the BOOT button to clear the drawing),
- shows a live on-screen status read-out: display panel, touch coordinates, SD
  card size, Ethernet IP, **RTPS publisher** status, and **system** info (free
  internal/PSRAM heap and uptime),
- mounts the microSD card (if inserted),
- initializes the ES8311 audio codec (and loads an embedded `click.wav`),
- brings up Ethernet (IP101) with DHCP and, once it has an IP, starts an
  **RTPS participant** that publishes a counter on `espp/test/counter`, and
- wires the BOOT button.

> [!IMPORTANT]
> The LCD adapter board's `RST_LCD` and `PWM` signals must be jumpered to the
> ESP32-P4 main board (`RST_LCD` → J1 GPIO27, `PWM` → J1 GPIO26) or the screen
> stays black. See the component README.

## Configuration

Use `idf.py menuconfig` → *ESP32-P4 Function EV Board Configuration* to select
the display panel (EK79007 1024x600 by default, or ILI9881C 800x1280) and adjust
task stack sizes / enable Ethernet.

## Build and Flash

```sh
idf.py set-target esp32p4
idf.py build flash monitor
```

## Notes

- PSRAM is required (enabled in `sdkconfig.defaults`) for the MIPI-DSI frame
  buffers.
- Plug an Ethernet cable to see the DHCP-assigned IP appear on screen and in the
  log.
