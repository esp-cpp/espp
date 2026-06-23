# MotorGo Plink Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/motorgo-plink/badge.svg)](https://components.espressif.com/components/espp/motorgo-plink)

The MotorGo Plink is a four-channel ESP32-S3 motor-control board from
Every Flavor Robotics. It combines:

- four dual-PWM DC motor outputs
- four MT6701-compatible SSI encoder chip-selects on a shared bus
- four RC-servo signal pins
- one onboard LSM6DS33 IMU on the internal I2C bus
- a Qwiic I2C bus plus a second internal I2C bus
- user and status LEDs

The `espp::MotorGoPlink` component provides a singleton board abstraction for
those documented pin mappings, plus convenient helpers for:

- initializing and driving the four DC-motor channels with normalized speed
  commands backed by `espp::BdcDriver` on MCPWM, where positive commands drive
  `pwm_a`, negative commands drive `pwm_b`, and zero disables both outputs
- initializing the shared encoder SPI/SSI bus and creating four
  `espp::Mt6701<SSI>` encoder instances
- initializing the onboard LSM6DS33 IMU on the hidden I2C bus via the shared
  `espp::Lsm6dso` driver
- accessing the four servo signal pins so you can attach your preferred
  servo-control driver
- controlling the user/status LEDs with simple brightness setters or a Gaussian
  breathing effect
- accessing the external Qwiic and internal I2C buses

## Example

The [example](./example) initializes the board, logs the MotorGo Plink pin map,
starts the onboard LEDs breathing, initializes the onboard IMU, and safely
keeps the motors stopped by default. Optional `menuconfig` flags let you enable
encoder polling and a stronger motor sweep while the LEDs continue pulsing,
allowing you to exercise the board interfaces together on real hardware.
