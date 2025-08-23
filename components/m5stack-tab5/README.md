# M5Stack Tab5 Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/m5stack-tab5/badge.svg)](https://components.espressif.com/components/espp/m5stack-tab5)

The M5Stack Tab5 is a highly expandable, portable smart-IoT terminal development device featuring a dual-chip architecture with rich hardware resources. The main controller uses the **ESP32-P4** SoC based on the RISC-V architecture with 16 MB Flash and 32 MB PSRAM. The wireless module uses the ESP32-C6-MINI-1U, supporting Wi-Fi 6.

The `espp::M5StackTab5` component provides a singleton hardware abstraction for initializing and managing all the Tab5's subsystems including display, touch, audio, camera, IMU, power management, and expansion interfaces.

## Key Features

### Display & Touch
- 5″ 1280 × 720 IPS TFT screen via MIPI-DSI
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

## Example

The [example](./example) shows how to use the `espp::M5StackTab5` hardware abstraction component to initialize and use various subsystems of the Tab5.

## Usage

```cpp
#include "m5stack-tab5.hpp"

// Get the singleton instance
auto &tab5 = espp::M5StackTab5::get();

// Initialize display
tab5.initialize_display();

// Initialize touch with callback
tab5.initialize_touch([](const auto &touch_data) {
    fmt::print("Touch at ({}, {})\n", touch_data.x, touch_data.y);
});

// Initialize audio system
tab5.initialize_audio();
tab5.volume(75.0f);  // Set volume to 75%

// Initialize camera
tab5.initialize_camera([](const uint8_t *data, size_t length) {
    fmt::print("Camera frame: {} bytes\n", length);
});

// Initialize IMU
tab5.initialize_imu();

// Initialize battery monitoring
tab5.initialize_battery_monitoring();
auto battery_status = tab5.get_battery_status();
fmt::print("Battery: {:.2f}V, {:.1f}mA, {}%\n", 
           battery_status.voltage_v, 
           battery_status.current_ma,
           battery_status.charge_percent);

// Initialize expansion interfaces
tab5.initialize_rs485(115200, true);  // 115200 baud with termination
tab5.initialize_sd_card();
tab5.initialize_wireless();
```

## API Overview

### Display & Touch
- `initialize_display()` - Initialize MIPI-DSI display
- `initialize_touch()` - Initialize GT911 multi-touch
- `brightness()` - Control backlight brightness
- `touchpad_read()` - LVGL integration helper

### Audio System
- `initialize_audio()` - Initialize dual audio codecs
- `volume()` / `mute()` - Audio control
- `play_audio()` - Audio playback
- `start_audio_recording()` - Voice recording

### Camera
- `initialize_camera()` - Initialize SC2356 camera
- `start_camera_capture()` - Begin video capture
- `take_photo()` - Capture single frame

### Sensors
- `initialize_imu()` - Initialize BMI270 IMU
- `initialize_rtc()` - Initialize real-time clock
- `set_rtc_wakeup()` - Program wake-up alarms

### Power Management
- `initialize_battery_monitoring()` - Enable power monitoring
- `get_battery_status()` - Read battery status
- `enable_battery_charging()` - Control charging
- `set_power_mode()` - Power optimization

### Communication
- `initialize_rs485()` - Industrial RS-485 interface
- `initialize_sd_card()` - microSD card support
- `initialize_usb_host()` / `initialize_usb_device()` - USB functionality
- `initialize_wireless()` - ESP32-C6 wireless module

### Buttons & GPIO
- `initialize_reset_button()` / `initialize_boot_button()` - Button handling
- Grove, M5-Bus, and STAMP expansion support

## Configuration

The component can be configured through menuconfig:

```
Component config → M5Stack Tab5 Configuration
```

Available options:
- Interrupt stack size
- Audio task stack size  
- Enable/disable wireless module
- Enable/disable camera support
- Enable/disable battery monitoring

## Hardware Connections

The BSP automatically handles all internal connections based on the Tab5's hardware design. External connections are available through:

- **Grove Connector**: GPIO53 (Yellow), GPIO54 (White), 5V, GND
- **M5-Bus**: Full 30-pin expansion with SPI, UART, I2C, GPIO, and power
- **STAMP Pads**: Reserved for Cat-M, NB-IoT, LoRaWAN modules
- **GPIO Extension**: Additional GPIO breakout
- **USB Ports**: Host (USB-A) and Device (USB-C)
- **RS-485**: Industrial communication interface

## Development Platforms

- **UiFlow2**: Visual programming environment
- **Arduino IDE**: Arduino framework support
- **ESP-IDF**: Native ESP-IDF development
- **PlatformIO**: Cross-platform IDE support

## Applications

- Smart home control panels
- Industrial HMI terminals  
- IoT development and prototyping
- Edge AI applications
- Remote monitoring systems
- Educational projects
- Portable measurement devices

## Notes

- Requires ESP32-P4 target (ESP-IDF 5.1+)
- Some features require additional configuration in menuconfig
- Battery monitoring requires INA226 component
- Camera functionality requires MIPI-CSI driver support
- Wireless features require ESP32-C6 communication setup

## License

This component is provided under the same license as the ESP-CPP project. 