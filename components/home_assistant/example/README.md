# Home Assistant Example

This example connects to WiFi in station mode, registers Home Assistant MQTT
discovery entities, publishes sample state, and conditionally demonstrates REST
and WebSocket API usage when credentials are configured.

## Demonstrated features

- WiFi STA startup
- MQTT discovery for a `sensor` and `switch`
- MQTT state publication
- Optional REST `GET /api/config`
- Optional WebSocket `subscribe_events`

## How to use example

Configure the example through menuconfig, then build and flash:

```sh
idf.py flash monitor
```

The example compiles without a running Home Assistant instance. At runtime:

- if WiFi is unavailable, it logs and skips network interactions
- if MQTT is not configured, it skips discovery/state publication
- if REST/WebSocket settings are not configured, it skips those calls
