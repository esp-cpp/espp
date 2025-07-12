# QMI8658 Example

[![Badge](https://components.espressif.com/components/espp/qmi8658/badge.svg)](https://components.espressif.com/components/espp/qmi8658)

The `Qmi8658` component provides a driver for the QMI8658 6-Axis
Inertial Measurement Units (IMUs).

It supports configuring and reading:
* 3-axis acceleration data
* 3-axis gyroscope data

It also supports providing a filter function for computing orientation data from
the raw sensor data.

## Example

The [example](./example) shows how to use the `espp::Qmi8658` component to initialize and
communicate with an QMI8658 6-axis IMU.

