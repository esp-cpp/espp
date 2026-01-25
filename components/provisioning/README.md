# Provisioning Component

[![Badge](https://components.espressif.com/components/espp/provisioning/badge.svg)](https://components.espressif.com/components/espp/provisioning)

The `Provisioning` component provides a web-based WiFi configuration system
for ESP32 devices. It creates a temporary WiFi access point with an embedded
web server, allowing users to scan for available networks, test credentials,
and save configuration through a mobile-friendly interface.

## Features

- Creates temporary WiFi AP with configurable SSID and password
- Embedded web server with responsive HTML interface
- Network scanning with signal strength indication
- Manual SSID entry option for hidden networks
- Credential validation before saving (tests actual connection)
- Manages stored credentials (view, delete, reconnect)
- Automatic AP shutdown after successful provisioning
- NVS-based persistent storage
- Callbacks for provisioning events

## Example

The [example](./example) demonstrates how to use the `espp::Provisioning`
class to configure WiFi credentials via a web interface, with support for
scanning networks, testing connections, and managing stored credentials.
