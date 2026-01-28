```{include} ../../components/remote_debug/example/README.md
```

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

```
[RemoteDebug Example/I][0.123]: Starting Remote Debug Example
[RemoteDebug Example/I][0.456]: Connecting to WiFi: MyWiFiNetwork
[WifiSta/I][1.234]: got ip: 192.168.1.100
[RemoteDebug Example/I][1.235]: WiFi connected, IP: 192.168.1.100
[RemoteDebug/I][1.345]: Remote Debug server started on http://192.168.1.100:8080
[RemoteDebug Example/I][1.346]: Web interface available at: http://192.168.1.100:8080
[RemoteDebug Example/I][2.000]: Test log message 1
[RemoteDebug Example/I][3.000]: Test log message 2
```

## Web Interface

Navigate to the IP address shown in the console output (e.g., `http://192.168.1.100:8080`) to access:

### GPIO Control
- View all configured GPIOs with custom labels
- Configure each pin as input or output
- Control output states with High/Low buttons
- Real-time state display for all pins

### ADC Monitoring
- Live display of current voltage values
- Real-time plotting with configurable history
- Multiple channels displayed simultaneously
- Auto-updating graphs

### Console Logs (if enabled)
- View stdout output remotely
- ANSI color code support for formatted logs
- Auto-scrolling to latest entries
- Configurable buffer size

## Troubleshooting

### Logs not updating in real-time
Ensure `CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE=y` is set in sdkconfig or menuconfig under Component config → LittleFS.

### Web interface slow with multiple clients
- Increase `adc_batch_size` to reduce update frequency
- Reduce ADC sample rate if high precision isn't needed
- Adjust task priorities if competing with other high-priority tasks

### GPIO states not updating
- Verify GPIO pins are not used by other peripherals
- Check that direction is set correctly (input vs output)
- Ensure GPIO numbers are valid for your ESP32 variant
