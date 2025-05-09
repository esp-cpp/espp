# ICM42607 Example

[![Badge](https://components.espressif.com/components/espp/icm42607/badge.svg)](https://components.espressif.com/components/espp/icm42607)

The `Icm42607` component provides a driver for the ICM42607 and ICM42670 6-Axis
Inertial Measurement Units (IMUs).

It supports configuring and reading:
* 3-axis acceleration data
* 3-axis gyroscope data

It does not _yet_ support enabling the DMP to compute any advanced orientations
or other data.

## Example

The [example](./example) shows how to use the `espp::Icm42607` component to initialize and
communicate with an ICM42607 / ICM42670 6-axis IMU.

