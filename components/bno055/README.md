# BNO055 9-Axis Absolute Orientation IMU Component

[![Badge](https://components.espressif.com/components/espp/bno055/badge.svg)](https://components.espressif.com/components/espp/bno055)

The `Bno055` component provides a driver for the Bosch BNO055 9-axis absolute
orientation sensor over I2C. The BNO055 runs sensor fusion on its own
microcontroller, so the driver reads finished orientation data rather than
filtering raw samples on the ESP.

It supports configuring and reading:
* Operating mode (raw-only modes and the IMU, COMPASS, M4G, and NDOF fusion
  modes)
* 3-axis acceleration, angular rate, and magnetic field
* Fused orientation as Euler angles and as a quaternion
* Linear acceleration (gravity removed) and the gravity vector
* Die temperature and per-sensor calibration status

All outputs are read in a single 46-byte I2C burst and reported in SI units
(m/s^2, rad/s, rad, uT).

It does not _yet_ support saving and restoring calibration offsets, axis
remapping, or the interrupt pin.

## Example

The [example](./example) shows how to use the `espp::Bno055` component to
initialize a BNO055 and print its outputs.
