# ESP32-P4-WIFI6-DEV-KIT Board Support Package (BSP)

[![Badge](https://components.espressif.com/components/espp/esp32-p4-wifi6-dev-kit/badge.svg)](https://components.espressif.com/components/espp/esp32-p4-wifi6-dev-kit)

Board Support Package for the [Waveshare ESP32-P4-WIFI6-DEV-KIT](https://www.waveshare.com/wiki/ESP32-P4-WIFI6-DEV-KIT)
board — an ESP32-P4NRW32 (32 MB stacked PSRAM; 16 MB NOR flash per the product
page, though the wiki self-contradicts on 16 vs 32 MB — unverified on
hardware) with an onboard
ESP32-C6 co-processor for Wi-Fi 6 / Bluetooth 5 (LE) over SDIO (ESP-Hosted).
The `espp::Esp32P4Wifi6DevKit` class is a singleton hardware abstraction that
brings up the board's peripherals with a small, uniform API.

## Supported peripherals

<table>
<tr><th>Peripheral</th><th>Hardware</th><th>API</th></tr>
<tr><td>Display</td><td>MIPI-DSI panel (JD9365 / ILI9881C 10.1" 800x1280 or EK79007 7" 1024x600, selected via Kconfig)</td><td><code>initialize_lcd()</code>, <code>initialize_display()</code></td></tr>
<tr><td>Touch</td><td>GT911 capacitive multi-touch (polled; INT/RST not routed)</td><td><code>initialize_touch()</code></td></tr>
<tr><td>Camera</td><td>MIPI-CSI (OV5647 by default) via esp_video / V4L2, RGB565 frames</td><td><code>initialize_camera()</code>, <code>stop_camera()</code></td></tr>
<tr><td>Audio out</td><td>ES8311 codec + NS4150B amplifier over I2S</td><td><code>initialize_audio()</code>, <code>play_audio()</code>, <code>volume()</code></td></tr>
<tr><td>Microphone</td><td>Onboard analog mic through the ES8311 ADC (full-duplex)</td><td><code>initialize_microphone()</code></td></tr>
<tr><td>uSD / TF card</td><td>4-bit SDMMC, powered by the on-chip LDO</td><td><code>initialize_sdcard()</code></td></tr>
<tr><td>Ethernet</td><td>10/100 internal EMAC + IP101GRI RMII PHY (DHCP client/server); RJ45 with optional external PoE-module header</td><td><code>initialize_ethernet()</code></td></tr>
<tr><td>Wi-Fi 6 / BT5</td><td>Onboard ESP32-C6 over SDIO (ESP-Hosted)</td><td>Not part of the BSP — use <code>espressif/esp_hosted</code> + <code>espressif/esp_wifi_remote</code> and the standard <code>esp_wifi</code> API (see below)</td></tr>
</table>

The ES8311 codec, GT911 touch controller, and camera SCCB share a single
internal I2C bus (`internal_i2c()`, SDA=GPIO7 / SCL=GPIO8). The Ethernet
bring-up is delegated to the reusable `espp::Ethernet` component; this BSP
supplies the board-specific RMII pin mapping.

The MIPI-CSI camera pipeline (esp_video + esp_cam_sensor) is fetched by the IDF
component manager. PSRAM must be enabled (the camera capture buffers and display
framebuffers live in PSRAM). See the example's `sdkconfig.defaults` for the
required Kconfig options (PSRAM, the MIPI-CSI video device, and the sensor).

## Wi-Fi 6 / Bluetooth via the onboard ESP32-C6

The ESP32-P4 has no radio of its own; the board pairs it with an ESP32-C6
running the ESP-Hosted slave firmware, connected over SDIO (CLK=GPIO18,
CMD=GPIO19, D0-D3=GPIO14-17, C6 reset=GPIO54 — the ESP-Hosted defaults for the
ESP32-P4). To use Wi-Fi from your application, add the two managed components:

```
idf.py add-dependency "espressif/esp_wifi_remote"
idf.py add-dependency "espressif/esp_hosted"
```

then select `esp32c6` as the slave target (menuconfig: `Component config ->
Wi-Fi Remote -> choose slave target`) and use the standard `esp_wifi` API
unchanged. The BSP intentionally does not wrap this — ESP-Hosted hooks in below
`esp_wifi`, so there is no board-specific code to add beyond the defaults.

## Example

The [example](./example) brings up the display + touch, audio (record /
playback), the camera (live feed), the uSD card, and Ethernet, and drives an
LVGL GUI with Status / Audio / Camera tabs.
