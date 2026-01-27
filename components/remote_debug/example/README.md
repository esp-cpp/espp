# Remote Debug Example

This example demonstrates the `espp::RemoteDebug` component, providing a
web-based interface for GPIO control, real-time ADC monitoring, and console log
viewing.

## How to use example

### Hardware Required

This example can run on any ESP32 development board. For testing:
- Connect LEDs or other peripherals / inputs to GPIO pins
- Connect analog sensors to ADC-capable pins

### Configure the project

```
idf.py menuconfig
```

Navigate to `Remote Debug Example Configuration`:
- WiFi credentials (SSID and password)
- Server configuration (port, title)
- GPIO configuration (number of pins, pin numbers, labels)
- ADC configuration (channels, attenuation, labels, sample rate, buffer size)
- Console logging (enable/disable, buffer size)

**Important**: If enabling console logging, you must also set:
- Component config → LittleFS → `CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE=y`

This ensures real-time log updates on the web interface.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Access the Interface

1. Device connects to configured WiFi network
2. Check serial monitor for assigned IP address
3. Open web browser to `http://<device-ip>:<port>` (default port: 8080)
4. Use the interface to control GPIOs, monitor ADCs, and view console logs

## Example Output

```
I (380) Remote Debug Example: Starting Remote Debug Example
I (390) Remote Debug Example: Connecting to WiFi: MyNetwork
I (2450) Remote Debug Example: WiFi connected! IP: 192.168.1.105
I (2456) Remote Debug Example: Initialized 2 ADC channels
I (2461) Remote Debug Example: Remote Debug Server started
I (2462) Remote Debug Example: Web interface: http://192.168.1.105:8080
I (2463) Remote Debug Example: GPIO pins: 4 | ADC channels: 2
```

## Web Interface Features

- **GPIO Control**
  - Configure pins as input or output
  - Read current states in real-time
  - Set output pins HIGH or LOW
  - Visual state indicators

- **ADC Monitoring**  
  - Real-time plotting with automatic updates
  - Multiple channels displayed simultaneously
  - Voltage display (converted from raw values)
  - Configurable sample rate and buffer size

- **Console Log Viewer** (when enabled)
  - Live stdout output
  - ANSI color code support
  - Auto-scrolling display
  - Configurable buffer size

## API Endpoints

Programmatic access via JSON REST API:

```
GET  /data              - Batched update (GPIO states + ADC data)
GET  /gpio              - List all GPIOs and current states
POST /gpio/direction    - Set GPIO direction (input/output)
POST /gpio/set          - Set GPIO output state
GET  /adc               - Get current ADC values
GET  /logs              - Get console log content (if enabled)
```
