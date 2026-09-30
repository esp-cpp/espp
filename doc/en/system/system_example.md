# System Example

[![Badge](https://components.espressif.com/components/espp/system/badge.svg)](https://components.espressif.com/components/espp/system)

This example shows how to use the `espp::SystemService` and
`espp::MonitorService` components to expose device info / control and live
heap + task statistics over the native USB port (WebUSB + Web Serial) of an
ESP32-S3, for the hosted system console web app.

## How to use example

### Hardware Required

An ESP32-S3 board with its native USB port connected to the host.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

```
I (317) System Example: Starting system info + control example
I (327) System Example: System:
ESP32-S3 rev 0.2 (2 cores), ESP-IDF v6.1
app: system_example 1 built Sep 30 2026 12:34:56
partition: running 'factory', boot 'factory', OTA state n/a
reset: power-on; uptime 320 ms; MAC 34:85:18:xx:xx:xx
flash 8192 KiB, PSRAM 0 KiB, CPU 240 MHz, heap free 318412 (min 318412)
I (357) System Example: Reboot into the bootloader is supported on this chip
I (1077) System Example: Ready. Connect the native USB port and open the system console ...
```
