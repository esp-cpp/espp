# DNS Server Example

This example demonstrates the use of the `dns_server` component to create a simple DNS server that responds to all queries with a single IP address. This is commonly used for captive portal implementations.

## How to use example

### Hardware Required

This example can be run on any ESP32 development board.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

## Example Output

```
[DNS Server Example/I][0.739]: Starting DNS Server Example
[DNS Server Example/I][0.739]: Starting WiFi AP: ESP-DNS-Test
[DNS Server Example/I][0.889]: WiFi AP started successfully
[DNS Server Example/I][0.889]: Connect to SSID: ESP-DNS-Test with password: testpassword
[DNS Server Example/I][0.899]: AP IP Address: 192.168.4.1
[DNS Server Example/I][0.899]: Starting DNS server on 192.168.4.1:53
[DNS Server Example/I][0.909]: DNS server started successfully
[DNS Server Example/I][0.909]: All DNS queries will resolve to: 192.168.4.1
```

## Testing

1. Connect your phone or computer to the WiFi network "ESP-DNS-Test" with password "testpassword"
2. Try to ping any domain: `ping google.com` - all domains should resolve to 192.168.4.1
3. The captive portal detection should trigger automatically on most devices
