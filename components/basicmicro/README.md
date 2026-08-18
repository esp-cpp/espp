# Basicmicro (MCP / RoboClaw) Motor Controller Component

[![Badge](https://components.espressif.com/components/espp/basicmicro/badge.svg)](https://components.espressif.com/components/espp/basicmicro)

`espp::Basicmicro` is a driver for Basicmicro **MCP236 / MCP266** (and other
RoboClaw-family) brushed DC motor controllers speaking their **PACKET SERIAL**
protocol, typically over UART.

The component is transport-agnostic: it performs no I/O itself and instead
calls user-provided `write` / `read` functions for each transaction, so it
works over a UART driver, USB CDC, an RS-232 adapter, etc. All wire-format
logic (CRC16, packet building, reply validation, big-endian codecs) lives in
`include/detail/basicmicro_core.hpp`, a host-buildable core with zero ESP
dependencies that is unit-tested off-target.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Basicmicro (MCP / RoboClaw) Motor Controller Component](#basicmicro-mcp--roboclaw-motor-controller-component)
  - [Features](#features)
  - [Protocol](#protocol)
  - [API](#api)
  - [Example](#example)
  - [Testing](#testing)

<!-- markdown-toc end -->

## Features

- Duty-cycle drive (commands 32/33/34) and closed-loop speed drive in
  quadrature pulses per second (35/36/37), with optional acceleration ramps
  (38/39/40)
- Buffered speed / accel / distance motion commands (41-46) plus buffer-state
  readback (47)
- Encoder support: counts (16/17/78), speeds (18/19/79), reset (20) and
  encoder-mode readback (91)
- Velocity PID get/set with automatic 16.16 fixed-point conversion (28/29,
  55/56)
- Telemetry: firmware version (21), main/logic battery voltage (24/25), motor
  currents (49), motor PWMs (48), board temperatures (82/83) and unit status
  (90)
- Management: write settings to EEPROM (94), E-Stop reset (200)
- No exceptions; all methods report errors via `std::error_code`
- Thread-safe: each transaction (request + ACK/reply) is serialized by an
  internal mutex

## Protocol

The packet serial protocol (MCP Series User Manual, section 2.2):

- Write commands send `[Address, Command, Data..., CRC16]`; the controller
  replies with a single `0xFF` ACK byte only when the packet was valid.
- Read commands send `[Address, Command]` (no CRC); the reply is the data
  followed by a CRC16 computed over the *sent* address + command bytes plus the
  reply data.
- All multi-byte values (including the CRC) are big-endian ("high byte first").
- CRC16 is CRC-16/XMODEM (poly `0x1021`, init `0`, non-reflected).
- Error recovery: the controller discards a partial packet after a 10 ms
  inter-byte gap, so the configured receive timeout (>= 10 ms, default 20 ms)
  doubles as the recovery mechanism.

## API

See the [documentation](https://esp-cpp.github.io/espp/motor_control/basicmicro.html).

## Example

The [example](./example) shows how to wire the driver to the ESP-IDF UART
driver, read the firmware version / battery voltage / status, run a gentle
duty-cycle ramp on motor 1 with encoder readback, and stop.

## Testing

The wire core is host-buildable and unit-tested without ESP-IDF:

```sh
c++ -std=c++20 -I include -o /tmp/bm_test test/basicmicro_host_test.cpp && /tmp/bm_test
```
