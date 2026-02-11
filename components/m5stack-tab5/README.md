# M5Stack Tab5 Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/m5stack-tab5/badge.svg)](https://components.espressif.com/components/espp/m5stack-tab5)

The M5Stack Tab5 is a highly expandable, portable smart-IoT terminal development device featuring a dual-chip architecture with rich hardware resources. The main controller uses the **ESP32-P4** SoC based on the RISC-V architecture with 16 MB Flash and 32 MB PSRAM. The wireless module uses the ESP32-C6-MINI-1U, supporting Wi-Fi 6.

The `espp::M5StackTab5` component provides a singleton hardware abstraction for initializing and managing all the Tab5's subsystems including display, touch, audio, camera, IMU, power management, and expansion interfaces.

## Key Features

### Display & Touch
- 5″ 1280 × 720 IPS TFT screen via MIPI-DSI
- **Automatic display controller detection** (supports ILI9881 or ST7123)
- GT911 multi-touch controller (I²C) for smooth interaction
- Adjustable backlight brightness control

### Audio System
- Dual audio codecs: ES8388 + ES7210 AEC front-end
- Dual-microphone array for voice recognition
- 1W speaker + 3.5mm headphone jack
- Hi-Fi recording and playback capabilities

### Camera
- SC2356 2MP camera (1600 × 1200) via MIPI-CSI
- HD video recording and image processing
- Support for edge-AI applications

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
| Touch | GT911 multi-touch controller |
| Camera | SC2356 @ 2 MP (1600 × 1200) |
| Audio | ES8388 codec + ES7210 AEC |
| IMU | BMI270 6-axis (accelerometer + gyroscope) |
| Battery | NP-F550 2000mAh removable |
| Expansion | Grove, M5-Bus, STAMP pads, GPIO headers |

## Display Controller Auto-Detection

The M5Stack Tab5 hardware can be manufactured with one of two different MIPI-DSI display controllers:
- **ILI9881** (earlier hardware revisions)
- **ST7123** (newer hardware revisions)

The BSP automatically detects which display controller is present during initialization by:
1. Attempting to initialize with the ILI9881 driver first
2. If ILI9881 detection fails, falling back to ST7123 initialization
3. Logging the detected controller type for debugging

This means your application code works seamlessly across both hardware variants without any code changes. You can optionally query the detected controller type:

```cpp
auto& tab5 = espp::M5StackTab5::get();
tab5.initialize_lcd();

// Query the detected controller
auto controller_type = tab5.get_display_controller();
const char* controller_name = tab5.get_display_controller_name();
```

## Example

The [example](./example) shows how to use the `espp::M5StackTab5` hardware abstraction component to initialize and use various subsystems of the Tab5.

