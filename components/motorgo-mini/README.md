# MotorGo-Mini Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/motorgo-mini/badge.svg)](https://components.espressif.com/components/espp/motorgo-mini)

The MotorGo Mini is a small, low-cost, low-power motor controller that can be
used to control a two motors.

https://motorgo.net

It's pretty sweet and the component provides the implementation of the two
channel FOC motor controller, along with other peripheral classes such as the
ADC, LEDs, and I2C.

## Symmetric API (shared with MotorGo Axis)

`espp::MotorGoMini` and `espp::MotorGoAxis` expose the same zero-based,
index-based API so the same code can drive either board. The original 1-based
named accessors (`motor1()`, `init_motor_channel_1()`, `encoder1()`,
`default_motor1_config`, …) remain available for backwards compatibility.

The shared API (motor channels are zero-based: `0` == "Motor 1", `1` == "Motor 2"):

- Types: `Encoder`, `MotorDriver`, `BldcMotor`
- `num_motor_channels()`, `driver_default_power_supply_voltage()`,
  `driver_default_voltage_limit()`, `default_motor_dead_zone_ns()`
- `initialize_encoders(run_tasks = true)`, `initialize_motors(...)`
- `encoder(index)`, `motor_driver(index)`, `motor(index)`
- `reset_encoder_accumulator(index)`
- `motor_driver_enabled(index)`, `enable_motor_driver(index)`,
  `disable_motor_driver(index)`, `enable_all_motor_drivers()`,
  `disable_all_motor_drivers()`
- `default_motor_config(index)`, `initialize_motor(index, config)`

```cpp
// Works unchanged on MotorGo Mini or MotorGo Axis:
using Board = espp::MotorGoMini; // or espp::MotorGoAxis
auto &board = Board::get();
board.initialize_encoders();                 // start the encoder update task(s)
board.initialize_motors();                   // create the motor driver(s)
size_t index = 0;                            // 0 == Motor 1, 1 == Motor 2
auto config = board.default_motor_config(index);
// tweak config (PID gains, current limit, ...) here if desired
auto motor = board.initialize_motor(index, config); // shared_ptr<Board::BldcMotor>
auto driver = board.motor_driver(index);
```

`initialize_motor()` creates (and calibrates) the FOC motor controller for the
channel, initializing that channel's encoder and driver first if they have not
been initialized yet, so a single call is enough to get a ready-to-use motor.

## Example

This example demonstrates how to use the `espp::MotorGoMini` component to
initialize the hardware on the [MotorGo Mini board](https://motorgo.net) which
is connected to two encoders and two BLDC motors. It uses those hardware to
drive the motors and outputs the state as a CSV.

