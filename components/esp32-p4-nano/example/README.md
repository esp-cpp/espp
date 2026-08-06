# ESP32-P4-NANO Example

Brings up the Waveshare ESP32-P4-NANO board through the `espp::Esp32P4Nano` BSP
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
