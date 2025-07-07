# Iperf Menu Example

This example shows the use of the `espp::IperfMenu` component to measure the
throughput of a network (WiFi / ethernet) connection between an ESP chip and
another device running iperf.

This supports:

- client operation
- server operation
- ipv4/ipv6

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Configuration

This example uses WiFi (specifically WiFi STA), so you should use the `wifi`
command line interface (CLI) menu that the example outputs over the serial
monitor to configure the WiFi network.

Once you're connected, you can use the `iperf` menu to select the operation mode
(client or server) and set the parameters for the test.
