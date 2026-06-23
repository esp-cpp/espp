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

## Example

The [example](./example) initializes the board, logs the MotorGo Axis pin map,
starts the onboard LEDs breathing, initializes the onboard IMU, initializes the
two BLDC driver channels in a disabled state, and can optionally poll the two
encoder inputs.
