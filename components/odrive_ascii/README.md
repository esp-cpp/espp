# ODrive ASCII Protocol Component

[![Badge](https://components.espressif.com/components/espp/odrive_ascii/badge.svg)](https://components.espressif.com/components/espp/odrive_ascii)

`espp::OdriveAscii` implements a minimal, dependency-free server for the ODrive-compatible ASCII protocol. It parses incoming bytes into commands and produces response bytes for transmission by the caller. It does not perform any I/O itself and is transport-agnostic (UART, USB CDC, socket, etc.).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ODrive ASCII Protocol Component](#odrive-ascii-protocol-component)
  - [Features](#features)
  - [API](#api)
  - [Example](#example)
  - [Notes on odrivetool discovery](#notes-on-odrivetool-discovery)

<!-- markdown-toc end -->

## Features

- **Transport-agnostic**: feed bytes in, get response bytes out
- **Property registry**: register `r`/`w` paths with getters/setters (no exceptions; uses `std::error_code`)
- **High-rate commands**:
  - `p <axis> <pos> [vel_ff [torque_ff]]`
  - `v <axis> <vel> [torque_ff]`
  - `c <axis> <torque_nm>` torque command (Nm)
  - `t <axis> <goal_pos_turns>` trajectory goal (turns)
  - `f <axis>` feedback: returns `"<pos> <vel>\n"`
  - `es <axis> <abs_pos_turns>` set encoder absolute position (turns)
- **No hardware dependencies**: integrates via `std::function` callbacks
- **Thread-safe**: internal locking for buffer, registry, and callbacks

## API

Key class: `espp::OdriveAscii`
- `process_bytes(std::span<const uint8_t>) -> std::vector<uint8_t>`
- Register properties: `register_property`, `register_float_property`, `register_int_property`, `register_bool_property`
- Set callbacks: `on_position_command`, `on_velocity_command`, `on_torque_command`, `on_trajectory_command`, `on_feedback_request`, `on_encoder_set_absolute`

See header [`include/odrive_ascii.hpp`](./include/odrive_ascii.hpp) and docs for details.

## Example

A full interactive example is provided in [`example`](./example) and is built by CI.

## Notes on odrivetool discovery

Auto-discovery in `odrivetool` relies on the Fibre protocol over USB vendor interface. This component intentionally implements ASCII only; you will need to specify a serial port in Python. Implementing Fibre will be done in a separate component in the future.
