# ESP32-P4-WIFI6-DEV-KIT Example

Brings up the Waveshare ESP32-P4-WIFI6-DEV-KIT board through the `espp::Esp32P4Wifi6DevKit` BSP
and drives an LVGL GUI. It exercises the display, touch, audio, microphone,
camera, uSD card and Ethernet:

- **Display + touch (MIPI-DSI + GT911):** a tabview GUI. On the **Status** tab,
  touch the screen to draw circles and play a click; it also shows live
  subsystem state (panel, touch, SD, Ethernet, camera size, memory / uptime).
  The rotate / clear buttons rotate the display and clear the drawing.
- **Audio (ES8311 + NS4150B):** the **Audio** tab has record / play buttons and
  speaker / microphone volume controls. Recording captures from the onboard
  microphone (full-duplex through the codec) into a PSRAM buffer; play streams it
  back to the speaker.
- **Camera (MIPI-CSI, OV5647):** the **Camera** tab shows the live RGB565 feed.
  Each frame from the BSP capture task is copied into an LVGL canvas.
- **uSD / TF card (4-bit SDMMC):** mounted at boot (if a card is present); its
  size is logged and shown on the Status tab.
- **Ethernet (internal EMAC + IP101GRI RMII PHY):** started as a DHCP client; the
  acquired IP is logged and shown on the Status tab.

Wi-Fi 6 (via the onboard ESP32-C6) is not exercised by this example — see
below for how to enable it in your own project.

## Wi-Fi 6 via the onboard ESP32-C6 (ESP-Hosted)

The ESP32-P4 has no radio; the board's ESP32-C6 (running the ESP-Hosted slave
firmware it ships with) provides Wi-Fi 6 / BT5 over SDIO (CLK=GPIO18,
CMD=GPIO19, D0-D3=GPIO14-17, C6 reset=GPIO54 — the ESP-Hosted defaults for the
ESP32-P4, so no pin configuration is needed). To add Wi-Fi to a project using
this BSP:

```
idf.py add-dependency "espressif/esp_wifi_remote"
idf.py add-dependency "espressif/esp_hosted"
```

then in `menuconfig` set `Component config -> Wi-Fi Remote -> choose slave
target` to `esp32c6`, and use the standard `esp_wifi` / `esp_netif` APIs
unchanged (e.g. ESP-IDF's `wifi/getting_started/station` example works as-is).
Note the C6 is 2.4 GHz-only. To (re)flash the C6's ESP-Hosted slave firmware,
see the [Waveshare wiki FAQ](https://www.waveshare.com/wiki/ESP32-P4-WIFI6-DEV-KIT)
(hold C6_IO9 low at power-on and flash via the C6 UART pads).

## How the camera feed reaches the GUI

`initialize_camera()` runs a capture task in the BSP that hands each RGB565 frame
to a callback. The example forwards it to the thread-safe
`Gui::set_camera_frame(data, w, h)`, which copies the frame into a PSRAM canvas
buffer (allocated lazily on the first frame / a size change) and invalidates the
LVGL canvas. The GUI's LVGL task then renders it on the Camera tab.

## Build and flash

```
idf.py set-target esp32p4
idf.py build flash monitor
```

The example uses a custom 16 MB flash partition table (`partitions.csv`) because
the display + LVGL + camera + audio application no longer fits the default 1 MB
app partition. PSRAM and the MIPI-CSI camera pipeline are enabled via
`sdkconfig.defaults`.
