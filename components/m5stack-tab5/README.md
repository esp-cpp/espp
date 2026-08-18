# M5Stack Tab5 Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/m5stack-tab5/badge.svg)](https://components.espressif.com/components/espp/m5stack-tab5)

The M5Stack Tab5 is a highly expandable, portable smart-IoT terminal development device featuring a dual-chip architecture with rich hardware resources. The main controller uses the **ESP32-P4** SoC based on the RISC-V architecture with 16 MB Flash and 32 MB PSRAM. The wireless module uses the ESP32-C6-MINI-1U, supporting Wi-Fi 6.

The `espp::M5StackTab5` component provides a singleton hardware abstraction for initializing and managing all the Tab5's subsystems including display, touch, audio, camera, IMU, power management, and expansion interfaces.

## Key Features

### Display & Touch
- 5″ 1280 × 720 IPS TFT screen via MIPI-DSI
- **Automatic display controller detection** (supports ILI9881, ST7123, or ST7121)
- Multi-touch (GT911 on ILI9881 units; integrated Sitronix TDDI touch on ST7123/ST7121 units)
- Adjustable backlight brightness control

### Audio System
- Dual audio codecs: ES8388 + ES7210 AEC front-end
- Dual-microphone array for voice recognition
- 1W speaker + 3.5mm headphone jack
- Hi-Fi recording and playback capabilities

### Camera
- MIPI-CSI camera (SC202CS) brought up via Espressif's `esp_video` (V4L2)
  pipeline (CSI receiver + ISP), with the ISP converting the sensor's RAW8
  output to RGB565
- `initialize_camera(callback)` streams frames to the callback from a capture
  task; the sensor's SCCB shares the internal I2C bus
- See the example's Camera tab for a live-feed display

### Sensors & IMU
- BMI270 6-axis sensor (accelerometer + gyroscope)
- Interrupt wake-up capability
- Real-time orientation and motion tracking

### Power Management
- Removable NP-F550 Li-ion battery
- MP4560 buck-boost converter
- IP2326 charge management
- INA226 real-time power monitoring
- Multiple power modes for efficiency

### Communication & Expansion
- ESP32-C6 wireless module (Wi-Fi 6, Thread, ZigBee)
- USB-A Host + USB-C OTG ports
- RS-485 industrial interface with switchable 120Ω terminator
- Grove and M5-Bus expansion headers
- microSD card slot
- STAMP expansion pads for additional modules

### Real-Time Clock
- RX8130CE RTC with timed interrupt wake-up
- Battery-backed time keeping
- Programmable wake-up alarms

## Hardware Specifications

| Component | Specification |
|-----------|---------------|
| Main SoC | ESP32-P4NRW32 (RISC-V 32-bit dual-core 400 MHz + LP single-core 40 MHz) |
| Wireless SoC | ESP32-C6-MINI-1U (Wi-Fi 6 @ 2.4 GHz / Thread / ZigBee) |
| Flash | 16 MB |
| PSRAM | 32 MB |
| Display | 5-inch IPS TFT (1280 × 720) |
| Touch | GT911 (ILI9881 units) / integrated Sitronix TDDI (ST7123/ST7121 units) |
| Camera | SC2356 @ 2 MP (1600 × 1200) |
| Audio | ES8388 codec + ES7210 AEC |
| IMU | BMI270 6-axis (accelerometer + gyroscope) |
| Battery | NP-F550 2000mAh removable |
| Expansion | Grove, M5-Bus, STAMP pads, GPIO headers |

## Display Controller Auto-Detection

The M5Stack Tab5 hardware has shipped with three different MIPI-DSI display revisions:
- **ILI9881** panel + separate GT911 touch controller (original units)
- **ST7123** TDDI — integrated display + touch (units manufactured after Oct 2025)
- **ST7121** TDDI — integrated display + touch (newest units)

The BSP automatically detects which display controller is present during initialization:
1. It polls the Sitronix TDDI touch interface (I²C `0x55`) for its firmware
   version register: **1 → ST7121**, **3 → ST7123**. (The two ST parts need
   different init sequences and DSI lane rates and cannot be told apart by
   their DSI ID.)
2. If instead a GT911 touch controller ACKs (I²C `0x14`), the unit is the
   original **ILI9881** revision.
3. The detected controller type is logged for debugging.

This means your application code works seamlessly across all three hardware variants without any code changes. You can optionally query the detected controller type:

```cpp
auto& tab5 = espp::M5StackTab5::get();
tab5.initialize_lcd();

// Query the detected controller
auto controller_type = tab5.get_display_controller();
const char* controller_name = tab5.get_display_controller_name();
```

## Example

The [example](./example) shows how to use the `espp::M5StackTab5` hardware abstraction component to initialize and use various subsystems of the Tab5.

