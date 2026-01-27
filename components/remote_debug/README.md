# Remote Debug

Web-based remote debugging interface providing GPIO control, real-time ADC monitoring, and optional console log viewing over HTTP.

## Features

- **GPIO Control**: Configure pins as input/output, read states, control outputs via web interface
- **ADC Monitoring**: Real-time visualization of up to 8 ADC channels with configurable sample rates
- **Console Log Viewer**: Optional stdout redirection to web-viewable log with ANSI color support
- **Clean API**: RESTful JSON endpoints for programmatic access
- **Responsive UI**: Modern web interface that works on desktop and mobile

## Usage

```cpp
#include "remote_debug.hpp"

// Configure GPIOs to expose
std::vector<espp::RemoteDebug::GpioConfig> gpios = {
    {.pin = 2, .label = "LED"},
    {.pin = 4, .label = "Button"}
};

// Configure ADC channels to monitor
std::vector<espp::RemoteDebug::Adc1ChannelConfig> adc1_channels = {
    {.channel = ADC1_CHANNEL_0, .atten = ADC_ATTEN_DB_12, .label = "Battery Voltage"}
};

// Create remote debug server
espp::RemoteDebug::Config config{
    .server_address = "0.0.0.0",
    .server_port = 8080,
    .title = "My Device Debug",
    .gpios = gpios,
    .adc1_channels = adc1_channels,
    .adc_sample_rate = 100.0f,  // Hz
    .adc_buffer_size = 1000,
    .enable_logging = true,  // Enable console log viewer
    .log_buffer_size = 4096,
    .log_level = espp::Logger::Verbosity::INFO
};

espp::RemoteDebug debug(config);
debug.start();

// Access at http://<device-ip>:8080
```

## Console Logging

When `enable_logging` is true, the component redirects `stdout` to a file that can be viewed in the web interface. The log viewer supports ANSI color codes for styled output.

**Important**: For real-time log updates, you must enable LittleFS file flushing:

```
CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE=y
```

Set this in your `sdkconfig.defaults` or via `idf.py menuconfig` → Component config → LittleFS.

Without this setting, logs will only appear after the file buffer is full or the file is closed.

## API

See the [Remote Debug API documentation](https://esp-cpp.github.io/espp/html/classespp_1_1_remote_debug.html) for detailed information.

## Example

See the [remote_debug example](example/README.md) for a complete working example with WiFi connection, GPIO control, ADC monitoring, and console log viewing.
