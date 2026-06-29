# MotorGo Axis Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/motorgo-axis/badge.svg)](https://components.espressif.com/components/espp/motorgo-axis)

The MotorGo Axis is an ESP32-S3 motor-control board from Every Flavor
Robotics. It combines:

- two 6-PWM BLDC motor outputs
- two MT6701-compatible SSI encoder chip-selects on a shared bus
- one onboard LSM6DS33 IMU on the internal I2C bus
- a Qwiic I2C bus plus a second internal I2C bus
- user and status LEDs

The `espp::MotorGoAxis` component provides a singleton board abstraction for
those documented pin mappings, plus convenient helpers for:

- initializing the two BLDC motor driver channels with `espp::BldcDriver`
- initializing the shared encoder SPI/SSI bus and creating two
  `espp::Mt6701<SSI>` encoder instances
- initializing the onboard LSM6DS33 IMU on the hidden I2C bus via the shared
  `espp::Lsm6dso` driver
- controlling the user/status LEDs with simple brightness setters or a Gaussian
  breathing effect
- accessing the external Qwiic and internal I2C buses

## Symmetric API (shared with MotorGo Mini)

`espp::MotorGoAxis` and `espp::MotorGoMini` expose the same zero-based,
index-based API so the same code can drive either board. All of the
`MotorGoAxis` methods listed below also exist on `MotorGoMini` (which
additionally keeps its original 1-based named accessors for backwards
compatibility).

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
// Works unchanged on MotorGo Axis or MotorGo Mini:
using Board = espp::MotorGoAxis; // or espp::MotorGoMini
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
This is new on MotorGo Axis: the board now owns the `BldcMotor` objects, in
addition to the per-channel `motor_driver(index)` / `encoder(index)` helpers it
already exposed.

## Example

The [example](./example) initializes the board, logs the MotorGo Axis pin map,
starts the onboard LEDs breathing, initializes the onboard IMU, initializes the
two BLDC driver channels in a disabled state, and can optionally poll the two
encoder inputs.
