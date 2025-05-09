# ICM20948 9-Axis IMU Component

[![Badge](https://components.espressif.com/components/espp/icm20948/badge.svg)](https://components.espressif.com/components/espp/icm20948)

The `Icm20948` component provides a driver for the ICM20948 9-Axis Inertial
Measurement Unit (IMU). 

It supports configuring and reading:
* 3-axis acceleration data
* 3-axis gyroscope data
* 3-axis magnetometer data

It does not _yet_ support enabling the DMP to compute filtered quaternions.

## Example

The [example](./example) shows how to use the `espp::Icm20948` component to
initialize and communicate with an ICM20948 9-axis IMU.

