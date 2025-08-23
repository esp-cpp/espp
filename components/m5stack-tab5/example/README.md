# M5Stack Tab5 BSP Example

This example demonstrates the comprehensive functionality of the M5Stack Tab5 development board using the `espp::M5StackTab5` BSP component. It showcases all major features including display, touch, audio, camera, IMU, power management, and communication interfaces.

## Features Demonstrated

### Core Systems
- **5" 720p MIPI-DSI Display**: Initialization and brightness control
- **GT911 Multi-Touch Controller**: Touch event handling with callbacks
- **Dual Audio System**: ES8388 codec + ES7210 AEC with recording/playback
- **SC2356 2MP Camera**: Photo capture and video streaming
- **BMI270 6-axis IMU**: Real-time motion sensing
- **Battery Management**: INA226 power monitoring with charging control

### Communication Interfaces
- **RS-485 Industrial Interface**: Bidirectional communication with termination control  
- **microSD Card**: File system support with SDIO interface
- **USB Host/Device**: USB-A host and USB-C OTG functionality
- **ESP32-C6 Wireless**: Wi-Fi 6, Thread, and ZigBee support
- **Real-Time Clock**: RX8130CE with alarm and wake-up features

### Expansion & GPIO
- **Grove Connector**: Standard Grove interface support
- **M5-Bus**: Full 30-pin expansion connector
- **STAMP Pads**: Reserved for additional modules
- **Button Handling**: Reset, boot, and power button support

## Hardware Requirements

- M5Stack Tab5 development board
- microSD card (optional)
- NP-F550 battery (included with Tab5 Kit)
- USB-C cable for programming and power

## How to Build and Flash

### Prerequisites

- ESP-IDF 5.1 or later with ESP32-P4 support
- Configured ESP-IDF environment

### Build Steps

1. Clone the repository and navigate to the example:
```bash
cd espp/components/m5stack-tab5/example
```

2. Set the target to ESP32-P4:
```bash
idf.py set-target esp32p4
```

3. Configure the project (optional):
```bash
idf.py menuconfig
```

4. Build the project:
```bash
idf.py build
```

5. Flash to the Tab5:
```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with your Tab5's serial port (e.g., `/dev/ttyUSB0` on Linux or `COM3` on Windows).

## Example Behavior

### Initialization Sequence
The example initializes all Tab5 subsystems in sequence:
1. Display system with 75% brightness
2. Touch controller with interrupt-driven callbacks
3. Audio system with 60% volume
4. Camera with 800x600 capture resolution
5. IMU for motion sensing
6. Battery monitoring for power management
7. Communication interfaces (RS-485, SD, USB, Wireless)
8. Real-time clock with current time display
9. Button handlers for user interaction

### Runtime Features

**Touch Interaction:**
- Touch events trigger audio click sounds
- Every 5th touch captures a photo
- Touch coordinates and count are logged

**Audio System:**
- Click sound playback on touch events
- Button-triggered audio recording toggle
- Volume and mute control

**Battery Monitoring:**
- Real-time voltage, current, and charge percentage
- Automatic low-power mode when battery < 20%
- Charging status detection

**IMU Data:**
- Continuous accelerometer and gyroscope readings
- Motion-based wake-up capability
- Real-time orientation tracking

**Communication Testing:**
- RS-485 test messages every 10 seconds
- SD card information display
- Wireless module status monitoring

**Status Reporting:**
- System status summary every 30 seconds
- Touch and photo counters
- Memory usage monitoring
- Individual subsystem health checks

### Expected Output

```
I (123) tab5_example: Starting M5Stack Tab5 BSP Example
I (124) tab5_example: ESP-IDF Version: v5.1.0
I (125) tab5_example: Free heap: 523456 bytes
I (126) tab5_example: === M5Stack Tab5 BSP Example ===
I (127) tab5_example: Display: 1280x720 pixels
I (128) tab5_example: Initializing display...
I (129) M5StackTab5: Initializing MIPI-DSI display (1280x720)
I (130) tab5_example: Display initialized - brightness: 75.0%
I (131) tab5_example: Initializing touch controller...
I (132) M5StackTab5: Initializing GT911 multi-touch controller
I (133) tab5_example: Touch controller initialized
...
I (200) tab5_example: === Initialization Complete ===
I (250) tab5_example: Touch detected: (640, 360) - 1 points, state: 1
I (251) tab5_example: Battery: 3.70V, -150.0mA, 555.0mW, 75.0% (Discharging)
...
```

## Configuration Options

The example can be configured through `menuconfig`:

```
Component config → M5Stack Tab5 Configuration
```

Available options:
- Interrupt stack size (default: 4096 bytes)
- Audio task stack size (default: 8192 bytes)
- Enable/disable wireless module
- Enable/disable camera support
- Enable/disable battery monitoring

## Troubleshooting

### Common Issues

**Display not working:**
- Ensure MIPI-DSI connections are secure
- Check power supply voltage (should be 5V)
- Verify ESP32-P4 MIPI-DSI driver support

**Touch not responding:**
- Check GT911 I2C connections (SDA/SCL)
- Verify interrupt pin configuration
- Ensure pull-up resistors on I2C lines

**Audio issues:**
- Check ES8388/ES7210 I2C addresses
- Verify I2S pin connections
- Ensure audio power enable is working

**Camera not working:**
- Check MIPI-CSI connections
- Verify SC2356 I2C communication
- Ensure camera power and reset signals

**Battery monitoring issues:**
- Check INA226 I2C communication
- Verify shunt resistor connections
- Ensure proper power management IC setup

### Debug Tips

1. Enable verbose logging:
```bash
idf.py menuconfig
# Component config → Log output → Default log verbosity → Verbose
```

2. Check I2C device detection:
```bash
# Add I2C scanner code to detect connected devices
```

3. Monitor power consumption:
```bash
# Use INA226 readings to verify power draw
```

4. Verify GPIO configurations:
```bash
# Check pin assignments match Tab5 hardware design
```

## Hardware Connections

The BSP automatically handles all internal connections. External connections available:

- **Grove (HY2.0-4P)**: GPIO53 (Yellow), GPIO54 (White), 5V, GND
- **M5-Bus**: Full 30-pin expansion with SPI, UART, I2C, GPIO, power
- **USB-A**: Host port for keyboards, mice, storage devices
- **USB-C**: Device/OTG port for programming and communication
- **RS-485**: Industrial communication (RX, TX, DIR control)
- **microSD**: Storage expansion via SDIO interface

## Performance Notes

- Display refresh rate: Up to 60 FPS at 720p
- Touch sampling rate: Up to 240 Hz
- Audio sample rates: 8kHz to 192kHz supported
- Camera frame rates: Up to 30 FPS at 1600x1200
- IMU update rate: Up to 1600 Hz
- Battery monitoring: 1 Hz continuous monitoring

## License

This example is provided under the same license as the ESP-CPP project. 