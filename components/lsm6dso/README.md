# LSM6DSO - 6-Axis IMU Driver (espp component)

[![Badge](https://components.espressif.com/components/espp/lsm6dso/badge.svg)](https://components.espressif.com/components/espp/lsm6dso)

This is an espp component for the LSM6DSO 6-axis IMU (3-axis accelerometer +
3-axis gyroscope) from STMicroelectronics. It supports both I2C and SPI
interfaces, FIFO, interrupts, tap/event detection, and on-chip filtering. The
driver is designed for use with the ESP-IDF and espp framework, and is modeled
after the ICM42607 and MT6701 components.

## Features
- Templated C++ driver supporting I2C and SPI
- Configurable accelerometer and gyroscope range and output data rate
- Orientation filtering (algorithmic and on-chip)
- FIFO, interrupts, tap and event detection support
- Example application for ESP-IDF

## Example

The [example](./example) shows how to use the `espp::Lsm6dso` component to
initialize and communicate with an LSM6DSO 6-axis IMU.

## Documentation
See the [documentation](https://esp-cpp.github.io/espp/imu/lsm6dso.html) for
full API details.

## License
See LICENSE file in the root of the repository. 
