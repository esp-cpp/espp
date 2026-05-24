# Home Assistant Component

[![Badge](https://components.espressif.com/components/espp/home_assistant/badge.svg)](https://components.espressif.com/components/espp/home_assistant)

The `home_assistant` component provides practical Home Assistant integration for
ESP-IDF applications using:

- MQTT discovery and runtime state / command topics
- Home Assistant REST API helpers
- Home Assistant WebSocket API connect / auth / subscribe helpers

The first release focuses on a useful subset of Home Assistant discovery fields
and a pragmatic API surface that works well for embedded devices.

## Features

- MQTT discovery support for:
  - `sensor`
  - `binary_sensor`
  - `button`
  - `switch`
  - `number`
  - `text`
  - `fan`
  - `cover`
  - `climate`
- Stable `unique_id` generation with grouped device metadata
- Retained discovery payload publication and retained removal support
- Command-topic callback dispatch for command-capable entities
- Synchronous REST helpers for `/api/config`, `/api/states`, and service calls
- WebSocket auth and raw JSON event/result callback support

## Usage

```cpp
#include "home_assistant.hpp"

std::error_code ec;

espp::HomeAssistant ha({
    .device =
        {
            .name = "Living Room Node",
            .identifiers = {"living-room-node"},
            .manufacturer = "espp",
            .model = "example",
            .sw_version = "1.0.0",
        },
    .mqtt =
        {
            .broker_uri = "mqtt://192.168.1.10",
            .discovery_prefix = "homeassistant",
            .state_prefix = "espp/home_assistant",
        },
    .api =
        {
            .base_url = "http://homeassistant.local:8123",
            .access_token = "<long-lived-access-token>",
        },
    .log_level = espp::Logger::Verbosity::INFO,
});

ha.register_sensor({
    .object_id = "temperature",
    .name = "Temperature",
    .unit_of_measurement = "°C",
}, ec);

ha.register_switch({
    .object_id = "relay",
    .name = "Relay",
    .on_command = [](std::string_view, std::string_view payload) {
      printf("switch command: %.*s\n", (int)payload.size(), payload.data());
    },
}, ec);

ha.start(ec);
ha.publish_entity_state("temperature", "23.4", ec);
ha.publish_entity_state("relay", "OFF", ec);
```

## Notes

- Discovery support intentionally covers a practical subset of fields for each
  entity type rather than every Home Assistant option.
- REST and WebSocket helpers expose raw JSON responses for flexibility.
- HTTPS / WSS support uses the ESP certificate bundle when enabled.

## Example

See [./example](./example) for a WiFi STA example that publishes MQTT discovery,
publishes entity state, and conditionally demonstrates REST and WebSocket calls.
