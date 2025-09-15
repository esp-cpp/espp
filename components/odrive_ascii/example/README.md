# ODrive ASCII Example

This example demonstrates how to use the `espp::OdriveAscii` component to process ODrive-compatible ASCII commands. It first runs a scripted test, then enters an interactive console mode so you can send commands from a serial terminal or Python.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ODrive ASCII Example](#odrive-ascii-example)
  - [Requirements](#requirements)
  - [Build](#build)
  - [Flash and Monitor](#flash-and-monitor)
  - [Interactive Usage](#interactive-usage)
  - [Notes](#notes)

<!-- markdown-toc end -->


## Requirements

- ESP-IDF installed and `get_idf` available in your shell
- A serial connection to the ESP target (USB Serial/JTAG or UART0)

## Build

```sh
# From repo root
cd components/odrive_ascii/example
get_idf
idf.py build
```

## Flash and Monitor

```sh
idf.py flash monitor
```

The example will:
- Run a scripted test sequence
- Enter interactive mode reading from stdin and writing responses to stdout

## Interactive Usage

Open a serial terminal or use Python to send commands:

```python
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

# Reads
ser.write(b'r axis0.encoder.pos_estimate\n'); print(ser.readline())
ser.write(b'r axis0.encoder.vel_estimate\n'); print(ser.readline())

# Writes
ser.write(b'w axis0.controller.input_pos 12.34\n'); print(ser.readline())

# High-rate commands
ser.write(b'p 0 1.0 0.5 0.1\n'); print(ser.readline())    # position with feedforward
ser.write(b'v 0 2.0 0.2\n'); print(ser.readline())        # velocity with torque_ff
ser.write(b'c 0 0.25\n'); print(ser.readline())           # torque (Nm)
ser.write(b't 0 10.0\n'); print(ser.readline())           # trajectory goal (turns)
ser.write(b'f 0\n'); print(ser.readline())                # feedback: "<pos> <vel>\n"
ser.write(b'es 0 0.0\n'); print(ser.readline())           # set encoder abs pos
```

## Notes

- Example config: `components/odrive_ascii/example/sdkconfig.defaults` sets 8KB main task stack and 115200 baud. Enable USB Serial/JTAG by uncommenting its line if needed.
- The example simulates motor state: position/velocity/torque are updated by incoming commands.
- For full odrivetool auto-discovery you would need to implement Fibre (not covered here).
