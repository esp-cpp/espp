# M5Stack Tab5 BSP Example

This example demonstrates the comprehensive functionality of the M5Stack Tab5 development board using the `espp::M5StackTab5` BSP component. It showcases all major features including display, touch, audio, camera, IMU, power management, and communication interfaces.

<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/124ae37f-c27f-4805-aa5b-79951d069b99" />

## Features Demonstrated

### Core Systems
- **5" 720p MIPI-DSI Display**: Initialization and brightness control
- **GT911 Multi-Touch Controller**: Touch event handling with callbacks
- **Dual Audio System**: ES8388 codec + ES7210 AEC with recording/playback
- **BMI270 6-axis IMU**: Real-time motion sensing
- **Battery Management**: INA226 power monitoring with charging control

### Communication Interfaces
- **microSD Card**: File system support with SDIO interface
- **Real-Time Clock**: RX8130CE with alarm and wake-up features

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
