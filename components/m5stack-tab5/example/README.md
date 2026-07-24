# M5Stack Tab5 BSP Example

This example demonstrates the comprehensive functionality of the M5Stack Tab5 development board using the `espp::M5StackTab5` BSP component. It showcases all major features including display, touch, audio, camera, IMU, power management, and communication interfaces.

<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/124ae37f-c27f-4805-aa5b-79951d069b99" />

## Features Demonstrated

### Core Systems
- **5" 720p MIPI-DSI Display**: Initialization and brightness control
- **GT911 Multi-Touch Controller**: Touch event handling with callbacks
- **Dual Audio System**: ES8388 codec + ES7210 AEC with recording/playback.
  The on-screen audio row (bottom-left) records from the microphones, plays
  the recording back through the speaker, and adjusts the speaker /
  microphone volumes; the recording buffer prefers PSRAM and the measured
  capture rate is logged when a recording stops. Because the speaker is
  mono, the recorded stereo (mic 1 / mic 2) is downmixed to both channels
  before playback, and the per-channel peak amplitude is logged so you can
  tell a silent capture (peak near 0) from a playback issue.
- **BMI270 6-axis IMU**: Real-time motion sensing
- **MIPI-CSI Camera (SC202CS)**: The **Camera** tab shows the live camera feed.
  The BSP brings up the camera via Espressif's `esp_video` (V4L2) pipeline
  (CSI + ISP, RAW8 -> RGB565) and streams frames to a capture task; each frame is
  downscaled and rotated to the current display orientation by the PPA, then the
  example copies it into an `lv_canvas` on the Camera tab. Pulls in the managed
  `espressif/esp_video` + `espressif/esp_cam_sensor` components (see
  `main/idf_component.yml`), so this example builds with the IDF component
  manager enabled. The ISP pipeline controller (auto exposure / white balance)
  is enabled so the feed is correctly white-balanced.
  - **Known upstream quirks (both benign, image unaffected):**
    - The stock SC202CS auto color-correction sometimes computes a CCM value
      beyond the ESP32-P4 ISP's `+/-4.0` limit under certain lighting. The ISP
      safely rejects it and keeps the previous CCM, but the pipeline logs the
      rejection every frame - enough blocking UART logging to drop frames - so
      the BSP's `initialize_camera()` silences those ISP/CCM log tags. See the
      comment in `components/m5stack-tab5/src/camera.cpp`.
    - You may also see occasional `ISP: gamma xcoord error` lines (a few at
      startup, then rarely on big scene-brightness changes). The precompiled
      auto-enhancement (AEN) hands the ISP a gamma table the hardware rejects;
      the ISP keeps the previous gamma, so the picture is fine and no frames are
      dropped. It is logged from an ISR (`ESP_EARLY_LOGE`), which ignores the
      runtime log-level controls, so it cannot be silenced the way the CCM spam
      is without also hiding real errors - it is left as-is.
- **Battery Management**: INA226 power monitoring with charging control

### Communication Interfaces
- **microSD Card**: File system support with SDIO interface
- **Real-Time Clock**: RX8130CE with alarm and wake-up features

## Configuration notes

This example raises some espp BSP task stack sizes above their component
defaults via `sdkconfig.defaults`, because the example does more work in
those tasks than the defaults assume:

- **Interrupt / touch task stack (`CONFIG_M5STACK_TAB5_INTERRUPT_STACK_SIZE` = 8192, up from the 4 KB
  BSP default).** The interrupt task services the touch controller over
  I2C; if an I2C transaction errors, the error is logged through espp's
  `fmt`-based (colorized) logger, whose formatting path needs several KB of
  stack. With only 4 KB this can overflow and corrupt memory. If you base
  your own project on this example (or reproduce its functionality), keep
  this override in your `sdkconfig` / `sdkconfig.defaults`.

## Hardware Requirements

- M5Stack Tab5 development board
- microSD card (optional)
- NP-F550 battery (included depending on your Tab5 Kit selection)
- USB-C cable for programming and power

## How to Build and Flash

### Prerequisites

- ESP-IDF 5.5 or later with ESP32-P4 support
- Configured ESP-IDF environment

### Build Steps

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

<img width="1001" height="1110" alt="CleanShot 2025-10-26 at 23 19 57" src="https://github.com/user-attachments/assets/dbb48431-6d0e-424d-a673-c12d8f10d12d" />
