# Remote Debug

[![Badge](https://components.espressif.com/components/espp/remote_debug/badge.svg)](https://components.espressif.com/components/espp/remote_debug)

Web-based remote debugging interface providing GPIO control, real-time ADC
monitoring, and optional console log viewing over HTTP. Uses `espp::Timer` for
efficient, configurable periodic updates.

https://github.com/user-attachments/assets/ee806c6c-0f6b-4dd0-a5b0-82b80410b5bc

<img width="607" height="2048" alt="image" src="https://github.com/user-attachments/assets/5977033c-eee8-4be2-a9c4-634fd3100480" />

## Features

- **GPIO Control**: Configure pins as input/output, read states, control outputs via web interface
- **ADC Monitoring**: Real-time visualization of ADC channels with configurable sample rates
- **Console Log Viewer**: Optional stdout redirection to web-viewable log with ANSI color support
- **Efficient Updates**: Uses `espp::Timer` for optimal performance with configurable priority
- **RESTful API**: JSON endpoints for programmatic access
- **Responsive UI**: Modern web interface that works on desktop and mobile
- **Multi-client Support**: Optimized for multiple concurrent clients through batched updates

## Performance

The component has been optimized for efficiency:

- Uses `espp::Timer` for precise, lightweight periodic updates
- Configurable task priority and stack size
- Batched ADC data updates reduce HTTP overhead
- Ring buffer implementation for efficient data management
- Efficient JSON generation minimizes processing overhead

## Dependencies

- `espp::Timer` - Periodic task execution
- `espp::Adc` - ADC channel management
- `espp::FileSystem` - LittleFS for optional log storage
- ESP HTTP Server - Web interface hosting

## Console Logging

When `enable_logging` is enabled in the config, stdout is redirected to a file
viewable in the web interface. The log viewer supports ANSI color codes.

**Important**: For real-time log updates, enable LittleFS file flushing:

```
CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE=y
```

Set this in your `sdkconfig.defaults` or via `idf.py menuconfig` → Component
config → LittleFS. Without this, logs only appear after the buffer fills.

## Example

See the example in the `example/` folder for a complete demonstration with WiFi
connection, GPIO control, ADC monitoring, and console log viewing.

## External Resources

- [ESP-IDF HTTP Server Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_server.html)
- [ADC Continuous Mode](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc_continuous.html)
