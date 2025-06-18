# LSM6DSO Example

This example demonstrates how to use the espp LSM6DSO 6-axis IMU driver with the
ESP-IDF. The example is modeled after the ICM42607 example and shows how to
configure the IMU, read accelerometer and gyroscope data, and use orientation
filtering (e.g., Madgwick filter).

![CleanShot 2025-06-18 at 14 16 01](https://github.com/user-attachments/assets/10b15539-688b-4711-b1ce-e7bb33f7e343)

## Features
- I2C communication with the LSM6DSO
- Configurable accelerometer and gyroscope range and output data rate
- Periodic reading of accelerometer, gyroscope, and temperature data
- Orientation filtering using Madgwick filter

## Usage
- Configure the I2C pins and address in `sdkconfig` or via Kconfig options
- Build and flash the example to your ESP32/ESP-IDF target
- The example will print IMU data and orientation to the serial console

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2025-06-18 at 14 15 13](https://github.com/user-attachments/assets/26b07fa0-4e4a-4025-a5a7-135322da8d3b)
![CleanShot 2025-06-18 at 14 16 01](https://github.com/user-attachments/assets/10b15539-688b-4711-b1ce-e7bb33f7e343)

## Example Code
See `main/lsm6dso_example.cpp` for the full example source code.

## Configuration
- Default I2C address: 0x6A (can be changed in Kconfig or via config struct)
- Example I2C pins: SDA = 21, SCL = 22

## Documentation
See the [documentation](https://esp-cpp.github.io/espp/imu/lsm6dso.html) for
full API details.
