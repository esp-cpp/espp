# ADXL345 Example

This example demonstrates how to use the `espp::Adxl345` class to read
acceleration data from an ADXL345 3-axis accelerometer over I2C using the
ESP-IDF framework and ESPP.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ADXL345 Example](#adxl345-example)
  - [Hardware Setup](#hardware-setup)
  - [Example Description](#example-description)
  - [Configuration](#configuration)
  - [Building and Flashing](#building-and-flashing)
  - [Customization](#customization)
  - [Output](#output)

<!-- markdown-toc end -->

## Hardware Setup

Connect the ADXL345 to your ESP32 (or S3, etc.) via I2C. Optionally, connect
INT1 to a GPIO for interrupt handling.

## Example Description

This example initializes the ADXL345, configures it for ±2g range and 100Hz data
rate, and continuously reads acceleration data (X, Y, Z axes) in a background
task. The data is printed in CSV format for easy plotting or analysis.

## Configuration

You can configure the I2C pins and other options in `sdkconfig.defaults` or via `menuconfig`:

- `CONFIG_EXAMPLE_I2C_SDA_GPIO` (default: 21)
- `CONFIG_EXAMPLE_I2C_SCL_GPIO` (default: 22)
- `CONFIG_EXAMPLE_ALERT_GPIO`   (default: 2)

## Building and Flashing

```sh
idf.py set-target esp32s3
idf.py menuconfig   # (optional) configure pins
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Customization

- To change the measurement range or data rate, modify the `range` and
  `data_rate` fields in the `espp::Adxl345::Config` struct in
  `adxl345_example.cpp`.
- For interrupt handling, connect INT1 to a GPIO

## Output

The example prints acceleration data in the following CSV format:

```
%time (s), x (g), y (g), z (g)
0.000, 0.0012, -0.0008, 1.0003
0.010, 0.0011, -0.0007, 1.0002
...
```

