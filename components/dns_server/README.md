# DNS Server

Simple DNS server component for implementing captive portals on ESP32 devices.

## Features

- Responds to all DNS queries with a configured IP address
- Lightweight implementation suitable for embedded systems
- Built on top of espp::UdpSocket for efficient UDP communication
- Useful for captive portal implementations where all domains should resolve to the device

## Usage

```cpp
#include "dns_server.hpp"

// Create DNS server that responds with the AP's IP
espp::DnsServer::Config dns_config{
  .ip_address = "192.168.4.1",
  .log_level = espp::Logger::Verbosity::INFO
};
espp::DnsServer dns_server(dns_config);

// Start the DNS server
if (dns_server.start()) {
  fmt::print("DNS server started\n");
} else {
  fmt::print("Failed to start DNS server\n");
}

// ... server runs in background ...

// Stop when done
dns_server.stop();
```

## How It Works

The DNS server listens on UDP port 53 (the standard DNS port) and responds to all A record queries with the configured IP address. This creates a "captive portal" effect where:

1. When a device connects to your WiFi AP, it tries to reach the internet
2. All DNS queries are answered with your device's IP address
3. The device's captive portal detection triggers
4. The user is directed to your web interface

## Integration with Provisioning

This component is designed to work seamlessly with the `espp::Provisioning` component to create a complete captive portal experience for WiFi provisioning.

## API

### Configuration

- `ip_address`: The IP address to respond with for all DNS queries (typically your AP's IP)
- `log_level`: Logging verbosity level

### Methods

- `start()`: Start the DNS server
- `stop()`: Stop the DNS server
- `is_running()`: Check if the server is currently running

## Limitations

- Only responds to A record queries (IPv4)
- Does not support AAAA records (IPv6)
- Minimal DNS implementation focused on captive portal use case
