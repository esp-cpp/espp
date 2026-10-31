# ESP32-P4-Module-DEV-KIT Board Support Package (BSP)

[![Badge](https://components.espressif.com/components/espp/esp32-p4-module-dev-kit/badge.svg)](https://components.espressif.com/components/espp/esp32-p4-module-dev-kit)

Board Support Package for the [Waveshare ESP32-P4-Module-DEV-KIT](https://www.waveshare.com/wiki/ESP32-P4-Module-DEV-KIT-StartPage)
board. The `espp::Esp32P4ModuleDevKit` class is a singleton hardware abstraction that
brings up the board's peripherals with a small, uniform API.

The dev kit is the Waveshare ESP32-P4-Module (ESP32-P4 + 16 MB flash + 32 MB
PSRAM + an on-module ESP32-C6 for WiFi 6 / Bluetooth 5) on a carrier board with
RJ45 Ethernet (optional PoE module), MIPI-DSI display and MIPI-CSI camera FFCs,
a TF-card slot, an ES8311 audio codec with on-board microphone and speaker
header, USB 2.0 OTG HS (Type-C + Type-A host), an RTC backup-battery connector,
and a GPIO pin header.

## Supported peripherals

<table>
<tr><th>Peripheral</th><th>Hardware</th><th>API</th></tr>
<tr><td>Display</td><td>MIPI-DSI panel, selected via Kconfig: JD9365 10.1" 800x1280 (default; the panel bundled with the -C kit), ILI9881C 10.1" 800x1280 (experimental), or EK79007 7" 1024x600</td><td><code>initialize_lcd()</code>, <code>initialize_display()</code></td></tr>
<tr><td>Touch</td><td>GT911 capacitive multi-touch (polled; INT/RST not routed)</td><td><code>initialize_touch()</code></td></tr>
<tr><td>Camera</td><td>MIPI-CSI (OV5647 by default) via esp_video / V4L2, RGB565 frames</td><td><code>initialize_camera()</code>, <code>stop_camera()</code></td></tr>
<tr><td>Audio out</td><td>ES8311 codec + NS4150B amplifier over I2S</td><td><code>initialize_audio()</code>, <code>play_audio()</code>, <code>volume()</code></td></tr>
<tr><td>Microphone</td><td>Onboard analog mic through the ES8311 ADC (full-duplex)</td><td><code>initialize_microphone()</code></td></tr>
<tr><td>uSD / TF card</td><td>4-bit SDMMC, powered by the on-chip LDO</td><td><code>initialize_sdcard()</code></td></tr>
<tr><td>Ethernet</td><td>10/100 internal EMAC + IP101GRI RMII PHY (DHCP client/server)</td><td><code>initialize_ethernet()</code></td></tr>
</table>

The ES8311 codec, GT911 touch controller, and camera SCCB share a single
internal I2C bus (`internal_i2c()`, SDA=GPIO7 / SCL=GPIO8). The Ethernet
bring-up is delegated to the reusable `espp::Ethernet` component; this BSP
supplies the board-specific RMII pin mapping.

The MIPI-CSI camera pipeline (esp_video + esp_cam_sensor) is fetched by the IDF
component manager. PSRAM must be enabled (the camera capture buffers and display
framebuffers live in PSRAM). See the example's `sdkconfig.defaults` for the
required Kconfig options (PSRAM, the MIPI-CSI video device, and the sensor).

## Not covered by this BSP

- **WiFi / Bluetooth:** provided by the ESP32-C6 on the ESP32-P4-Module over
  SDIO (esp_hosted default pins). Use the `esp_wifi_remote` + `esp_hosted`
  components directly.
- **USB:** the Type-C OTG HS port and the Type-A host ports are driven by the
  standard ESP-IDF USB host / device stacks.

## Example

The [example](./example) brings up the display + touch, audio (record /
playback), the camera (live feed), the uSD card, and Ethernet, and drives an
LVGL GUI with Status / Audio / Camera tabs.
