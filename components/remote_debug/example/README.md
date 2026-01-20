# Remote Debug Example

Web-based remote debugging interface for GPIO control and real-time ADC monitoring.

## How to Use

### Hardware Setup

Connect GPIOs and ADC channels you want to monitor/control. Default pins:
- GPIOs: 2, 4, 16, 17 (configurable)
- ADCs: 36, 39 (configurable)

Connect LEDs, sensors, or other peripherals to these pins for testing.

### Configure the Project

```bash
idf.py menuconfig
```

Navigate to `Remote Debug Example Configuration`:
- Set WiFi SSID and password
- Configure server port (default: 8080)
- Set number of GPIOs to expose (1-10)
- Configure which GPIO pins to use
- Set number of ADC channels (0-8)
- Configure which ADC pins to monitor
- Set ADC sample rate and buffer size

### Build and Flash

```bash
idf.py build flash monitor
```

### Access the Interface

1. Device connects to your WiFi network
2. Check serial monitor for IP address
3. Open browser to `http://<device-ip>:8080`
4. Use the web interface to control GPIOs and monitor ADCs

## Features

### GPIO Control

- **Read**: Get current state of GPIO pins
- **Write**: Set GPIO pins HIGH or LOW
- **Toggle**: Flip GPIO state
- **Visual indicators**: Shows current state in real-time

### ADC Monitoring

- **Real-time plotting**: Live graph of ADC values
- **Multiple channels**: Monitor up to 8 channels simultaneously
- **Configurable sample rate**: 1-1000 Hz
- **Voltage display**: Shows current values in volts
- **Time-series data**: Scrolling graph with configurable buffer

### JSON API

For programmatic access:
```
GET  /api/gpio         - List all GPIOs and states
GET  /api/gpio/<pin>   - Read specific GPIO
POST /api/gpio/<pin>   - Write GPIO (body: {"state": 1})
GET  /api/adc          - Get current ADC values
GET  /api/adc/data     - Get ADC plot data (all samples)
```

## Example Output

```
I (380) Remote Debug Example: Starting Remote Debug Example
I (385) Remote Debug Example: Connecting to WiFi SSID: MyWiFi
I (2450) Remote Debug Example: Got IP: 192.168.1.105
I (2451) Remote Debug Example: Connected to WiFi! IP: 192.168.1.105
I (2456) Remote Debug Example: Initialized 2 ADC channels
I (2461) Remote Debug Example: Remote Debug Server started!
I (2462) Remote Debug Example: Open browser to: http://192.168.1.105:8080
I (2463) Remote Debug Example: GPIO pins available: 4
I (2464) Remote Debug Example: ADC channels available: 2
```

## Use Cases

### Development & Testing
- Toggle GPIOs to control relays, LEDs, motors
- Monitor sensor values in real-time
- Debug analog circuits
- Test peripheral connections

### Remote Monitoring
- Monitor battery voltage
- Track temperature sensors
- Log environmental data
- Remote equipment status

### Interactive Demos
- Control devices from web browser
- Live sensor visualization
- Educational demonstrations
- Prototyping and proof-of-concept

## Web Interface Features

- **Clean, modern UI**: Responsive design for desktop and mobile
- **Real-time updates**: ADC plots update automatically
- **Color-coded states**: Visual feedback for GPIO states
- **Labeled pins**: Easy identification of each GPIO/ADC
- **Toggle buttons**: Quick GPIO control
- **Zoom/pan**: Interactive ADC plots

## Customization

### Adding Custom Controls

Modify the HTML/JavaScript in `remote_debug.cpp` to add:
- Custom widgets
- Additional data visualization
- Multi-GPIO patterns
- PWM control
- I2C/SPI device control

### Integration

The component can be integrated into any application that needs remote debugging capabilities. Just initialize with your GPIO/ADC configuration and start the server.

## Troubleshooting

- **Can't connect**: Check WiFi credentials and verify IP address
- **No ADC readings**: Verify GPIO numbers support ADC on your ESP32 variant
- **GPIO not responding**: Check pin isn't used by other peripherals
- **Slow updates**: Reduce ADC sample rate or buffer size
- **Port conflict**: Change server port if 8080 is in use

## Security Note

This is a development/debugging tool. For production use, add authentication and use HTTPS. The current implementation has no access control.
