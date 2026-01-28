# DNS Server Example

This example demonstrates using the `espp::DnsServer` component to create a DNS server that responds to all DNS queries. This is useful for creating captive portals or local network services.

## How to use example

### Hardware Required

This example can be run on any ESP32 development board.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![image](https://github.com/user-attachments/assets/dns_server_example_output.png)

The example will:
1. Start a WiFi Access Point
2. Start a DNS server that responds to all queries
3. Log all DNS requests and responses

You can test the DNS server by:
1. Connecting a device to the WiFi AP
2. Trying to access any domain (e.g., `ping google.com`)
3. All domains will resolve to the ESP32's IP address
