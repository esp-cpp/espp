# Motor Controller Component

[![Badge](https://components.espressif.com/components/espp/motor_controller/badge.svg)](https://components.espressif.com/components/espp/motor_controller)

Header-only shared interface for the espp dual-channel motor-controller drivers
that model the same MCP236 / MCP266 (Basicmicro / RoboClaw-family) hardware over
different transports. It defines:

- `espp::MotorAxis` — the `M1` / `M2` channel selector, and
- `espp::MotorController` — a compile-time concept describing the common command
  and read surface (`drive_duty` / `drive_speed` by axis, `read_encoder` /
  `read_speed`, `reset_estop`, `read_main_battery_voltage`, `read_temperature`).

Keeping the contract in a small, dependency-light component lets the two sibling
drivers share it without depending on each other: `espp::Basicmicro` (packet
serial) and `espp::Mcp266` (CANopen) each `static_assert` that they satisfy
`MotorController`, so generic code can command either transport by axis.

This is an interface / types component (like `bldc_types`): it has no runnable
example of its own — it is exercised by the `basicmicro` and `mcp266` examples,
which build in CI and instantiate the drivers that implement the concept.

## Example

```cpp
#include "motor_controller.hpp"

// Works for either espp::Basicmicro or espp::Mcp266 (both satisfy the concept).
bool ramp(espp::MotorController auto &mc, std::error_code &ec) {
  return mc.drive_duty(espp::MotorAxis::M1, 4096, ec);
}
```
