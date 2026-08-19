# Basicmicro (MCP / RoboClaw) Example

This example demonstrates how to use the `espp::Basicmicro` component to talk
to a Basicmicro MCP236 / MCP266 (or RoboClaw-family) motor controller over
UART using the packet serial protocol. It:

1. reads the firmware version, main battery voltage and unit status,
2. resets the encoders,
3. runs a gentle duty-cycle ramp on motor 1 while reading back the encoder
   count and speed, then ramps back down and stops, and
4. periodically logs the motor currents and board temperature.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Basicmicro (MCP / RoboClaw) Example](#basicmicro-mcp--roboclaw-example)
  - [Requirements](#requirements)
  - [Hardware](#hardware)
  - [Build](#build)
  - [Flash and Monitor](#flash-and-monitor)

<!-- markdown-toc end -->

## Requirements

- ESP-IDF installed and `get_idf` available in your shell
- A Basicmicro MCP / RoboClaw controller configured for **packet serial** mode
  at 38400 baud, address `0x80` (the defaults used by this example)

## Hardware

| ESP32 (this example) | MCP / RoboClaw |
|----------------------|----------------|
| GPIO 17 (UART1 TX)   | S1 (RX)        |
| GPIO 16 (UART1 RX)   | S2 (TX)        |
| GND                  | GND            |

Adjust the pins / port / baud rate at the top of
[basicmicro_example.cpp](./main/basicmicro_example.cpp) to match your wiring.

## Build

```sh
# From repo root
cd components/basicmicro/example
get_idf
idf.py set-target esp32
idf.py build
```

## Flash and Monitor

```sh
idf.py flash monitor
```
