# BMI270 Example

This example shows how to use the `espp::Bmi270` component to initialize and communicate with a BMI270 6-axis IMU.

<img width="822" height="930" alt="CleanShot 2025-08-09 at 17 22 30" src="https://github.com/user-attachments/assets/5ea18011-592d-4fa6-bdde-3a7afb84396b" />

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [BMI270 Example](#bmi270-example)
  - [Features Demonstrated](#features-demonstrated)
  - [How to use example](#how-to-use-example)
    - [Hardware Required](#hardware-required)
    - [Configuration](#configuration)
    - [Build and Flash](#build-and-flash)
  - [Example Output](#example-output)
    - [Sample Output](#sample-output)
    - [Data Analysis](#data-analysis)
  - [Troubleshooting](#troubleshooting)
    - [Common Issues](#common-issues)
    - [Debug Tips](#debug-tips)
  - [Performance Notes](#performance-notes)
  - [Advanced Features](#advanced-features)
  - [References](#references)

<!-- markdown-toc end -->


## Features Demonstrated

- **Basic IMU Reading**: Accelerometer, gyroscope, and temperature data
- **Orientation Filtering**: Both Kalman and Madgwick filter implementations
- **Real-time Data Logging**: CSV format output suitable for plotting
- **Interrupt Configuration**: Data ready and motion detection interrupts
- **Advanced Configuration**: Performance modes, bandwidth settings, power management

## How to use example

### Hardware Required

This example can run on any ESP32 development board with I2C pins available. Connect a BMI270 sensor as follows:

| BMI270 Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VDD        | 3.3V      | Power supply |
| VDDIO      | 3.3V      | I/O power supply |
| GND        | GND       | Ground |
| SCL        | GPIO22    | I2C clock (configurable) |
| SDA        | GPIO21    | I2C data (configurable) |
| SDO        | GND       | I2C address select (0x68) |
| INT1       | GPIO (optional) | Interrupt 1 output |

### Configuration

You can configure the I2C pins through menuconfig:

```
idf.py menuconfig
```

Navigate to: `BMI270 Example Configuration`

- **SCL GPIO Num**: I2C clock pin (default: 22). If running on M5Stack Tab5, set to 32.
- **SDA GPIO Num**: I2C data pin (default: 21). If running on M5Stack Tab5, set to 31.
- **I2C Clock Speed**: I2C frequency in Hz (default: 1000000)

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

The example outputs CSV-formatted data that includes:

1. **Timestamp** (seconds)
2. **Raw accelerometer data** (g)
3. **Raw gyroscope data** (°/s)
4. **Temperature** (°C)
5. **Kalman filter orientation** (radians)
6. **Kalman filter gravity vector**
7. **Madgwick filter orientation** (radians)
8. **Madgwick filter gravity vector**

### Sample Output

```
I (298) main_task: Calling app_main()
[BMI270 Example/I][0.023]: Starting example!
[BMI270 Example/I][0.028]: Found BMI270 at address: 0x68
[Bmi270/I][0.036]: Setting power mode to 4
[Bmi270/I][0.039]: Loading BMI270 configuration file (8192 B), this may take some time...
[Bmi270/I][0.140]: Configuration file loaded successfully
[Bmi270/I][0.141]: BMI270 initialized successfully
% Time (s), Accel X (g), Accel Y (g), Accel Z (g), Gyro X (°/s), Gyro Y (°/s), Gyro Z (°/s), Temp (°C), Kalman Roll (rad), Kalman Pitch (rad), Kalman Yaw (rad), Kalman Gravity X, Kalman Gravity Y, Kalman Gravity Z, Madgwick Roll (rad), Madgwick Pitch (rad), Madgwick Yaw (rad), Madgwick Gravity X, Madgwick Gravity Y, Madgwick Gravity Z
[BMI270 Example/I][0.170]: Starting task...
0.186,-0.005,-0.052,0.993,0.000,0.000,0.000,27.3,-0.052,0.006,0.000,0.006,0.052,-0.999,0.000,-0.000,0.000,-0.000,-0.000,-1.000
0.198,-0.005,-0.051,1.002,-0.213,-0.427,-0.335,27.4,-0.050,0.005,0.000,0.005,0.050,-0.999,-0.002,0.000,-0.000,0.000,0.002,-1.000
0.210,-0.004,-0.051,1.000,0.244,-0.579,-0.915,27.3,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.005,0.000,-0.000,0.000,0.005,-1.000
0.222,-0.004,-0.052,0.998,0.030,-0.122,-0.274,27.3,-0.052,0.004,0.000,0.004,0.052,-0.999,-0.007,0.000,-0.000,0.000,0.007,-1.000
0.234,-0.005,-0.051,0.999,0.000,-0.213,-0.213,27.3,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.010,0.001,-0.000,0.001,0.010,-1.000
0.246,-0.004,-0.051,0.998,0.030,-0.152,-0.183,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.012,0.001,-0.000,0.001,0.012,-1.000
0.258,-0.005,-0.051,0.999,0.061,-0.183,-0.183,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.014,0.001,-0.000,0.001,0.014,-1.000
0.270,-0.003,-0.051,1.000,0.061,-0.183,-0.183,27.4,-0.051,0.003,0.000,0.003,0.051,-0.999,-0.017,0.001,-0.000,0.001,0.017,-1.000
0.282,-0.005,-0.051,1.000,0.030,-0.183,-0.213,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.019,0.001,-0.001,0.001,0.019,-1.000
0.294,-0.003,-0.049,0.999,0.091,-0.183,-0.213,27.3,-0.049,0.003,0.000,0.003,0.049,-0.999,-0.021,0.001,-0.001,0.001,0.021,-1.000
0.306,-0.004,-0.049,0.999,0.061,-0.091,-0.274,27.4,-0.049,0.004,0.000,0.004,0.049,-0.999,-0.024,0.002,-0.001,0.002,0.024,-1.000
0.318,-0.005,-0.050,0.999,0.061,-0.213,-0.213,27.4,-0.050,0.005,0.000,0.005,0.050,-0.999,-0.026,0.002,-0.001,0.002,0.026,-1.000
0.330,-0.007,-0.052,0.999,0.091,-0.183,-0.244,27.4,-0.052,0.007,0.000,0.007,0.052,-0.999,-0.029,0.002,-0.001,0.002,0.029,-1.000
0.342,-0.006,-0.052,1.000,0.152,-0.152,-0.274,27.4,-0.052,0.006,0.000,0.006,0.052,-0.999,-0.031,0.003,-0.001,0.003,0.031,-1.000
0.354,-0.004,-0.051,1.002,0.061,-0.183,-0.244,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.033,0.003,-0.001,0.003,0.033,-0.999
0.366,-0.004,-0.052,1.000,0.061,-0.244,-0.213,27.4,-0.052,0.004,0.000,0.004,0.052,-0.999,-0.036,0.003,-0.001,0.003,0.036,-0.999
0.378,-0.004,-0.052,1.001,0.061,-0.183,-0.183,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.038,0.003,-0.001,0.003,0.038,-0.999
0.390,-0.003,-0.049,1.000,0.030,-0.152,-0.183,27.4,-0.049,0.003,0.000,0.003,0.049,-0.999,-0.040,0.003,-0.001,0.003,0.040,-0.999
0.402,-0.004,-0.051,0.999,0.061,-0.152,-0.213,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.043,0.003,-0.001,0.003,0.043,-0.999
0.414,-0.004,-0.051,1.000,0.152,-0.244,-0.305,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.045,0.003,-0.001,0.003,0.045,-0.999
0.426,-0.004,-0.052,1.000,0.030,-0.152,-0.183,27.4,-0.052,0.004,0.000,0.004,0.052,-0.999,-0.048,0.004,-0.001,0.004,0.047,-0.999
0.438,-0.005,-0.051,1.000,0.061,-0.152,-0.244,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.050,0.005,-0.001,0.005,0.050,-0.999
0.450,-0.005,-0.050,0.999,0.030,-0.122,-0.274,27.4,-0.050,0.005,0.000,0.005,0.050,-0.999,-0.052,0.006,-0.001,0.006,0.052,-0.999
0.462,-0.004,-0.050,1.000,0.000,-0.183,-0.213,27.4,-0.050,0.004,0.000,0.004,0.050,-0.999,-0.050,0.004,-0.001,0.004,0.050,-0.999
0.474,-0.005,-0.051,0.999,0.091,-0.244,-0.274,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.052,0.005,-0.001,0.005,0.052,-0.999
0.486,-0.004,-0.051,1.000,0.061,-0.183,-0.244,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.051,0.003,-0.001,0.003,0.051,-0.999
0.498,-0.004,-0.052,1.000,0.152,-0.274,-0.274,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.052,0.005,-0.001,0.005,0.052,-0.999
0.510,-0.006,-0.053,1.000,0.152,-0.122,-0.244,27.4,-0.053,0.006,0.000,0.006,0.053,-0.999,-0.054,0.006,-0.002,0.006,0.054,-0.999
0.522,-0.005,-0.052,1.000,0.030,-0.183,-0.244,27.4,-0.052,0.005,0.000,0.005,0.051,-0.999,-0.052,0.005,-0.002,0.005,0.052,-0.999
0.534,-0.003,-0.052,1.000,0.030,-0.152,-0.274,27.4,-0.052,0.003,0.000,0.003,0.052,-0.999,-0.053,0.003,-0.002,0.003,0.053,-0.999
0.546,-0.003,-0.051,0.999,0.061,-0.152,-0.335,27.4,-0.051,0.003,0.000,0.003,0.051,-0.999,-0.051,0.004,-0.002,0.004,0.051,-0.999
0.558,-0.006,-0.052,1.000,0.061,-0.122,-0.335,27.4,-0.052,0.006,0.000,0.006,0.052,-0.999,-0.052,0.006,-0.002,0.006,0.052,-0.999
0.570,-0.005,-0.051,1.001,0.030,-0.152,-0.274,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.050,0.004,-0.002,0.004,0.050,-0.999
0.582,-0.004,-0.050,1.001,0.061,-0.183,-0.274,27.4,-0.050,0.004,0.000,0.004,0.050,-0.999,-0.051,0.006,-0.002,0.006,0.051,-0.999
0.594,-0.004,-0.050,1.001,0.061,-0.244,-0.183,27.4,-0.050,0.004,0.000,0.004,0.050,-0.999,-0.050,0.004,-0.002,0.004,0.050,-0.999
0.606,-0.005,-0.052,0.999,0.091,-0.213,-0.244,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.052,0.005,-0.002,0.005,0.052,-0.999
0.618,-0.004,-0.052,1.000,0.091,-0.152,-0.152,27.4,-0.052,0.004,0.000,0.004,0.052,-0.999,-0.051,0.003,-0.002,0.003,0.051,-0.999
0.630,-0.005,-0.051,1.002,0.091,-0.274,-0.152,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.051,0.005,-0.002,0.005,0.051,-0.999
0.642,-0.005,-0.052,1.001,0.000,-0.213,-0.244,27.4,-0.051,0.005,0.000,0.005,0.051,-0.999,-0.053,0.003,-0.002,0.003,0.053,-0.999
0.654,-0.004,-0.052,1.001,0.030,-0.213,-0.305,27.4,-0.052,0.004,0.000,0.004,0.052,-0.999,-0.051,0.004,-0.002,0.004,0.051,-0.999
0.666,-0.004,-0.051,1.001,0.000,-0.152,-0.244,27.4,-0.051,0.004,0.000,0.004,0.051,-0.999,-0.049,0.004,-0.002,0.004,0.049,-0.999
0.678,-0.006,-0.051,1.000,0.152,-0.183,-0.274,27.4,-0.051,0.006,0.000,0.006,0.051,-0.999,-0.051,0.006,-0.002,0.006,0.051,-0.999
```

<img width="1905" height="902" alt="CleanShot 2025-08-09 at 17 06 50" src="https://github.com/user-attachments/assets/7784a5ee-41e3-466e-8b62-882f5be4ab5d" />
<img width="822" height="930" alt="CleanShot 2025-08-09 at 17 22 30" src="https://github.com/user-attachments/assets/5ea18011-592d-4fa6-bdde-3a7afb84396b" />
<img width="822" height="930" alt="CleanShot 2025-08-09 at 17 22 47" src="https://github.com/user-attachments/assets/358c05da-4dea-465c-8dc6-6ec3b9ca7cae" />

### Data Analysis

The CSV output can be imported into analysis tools like:

- **MATLAB/Octave**: For advanced signal processing
- **Python (pandas/matplotlib)**: For data visualization
- **Excel/LibreOffice**: For basic analysis
- **[esp-cpp/uart-serial-plotter](https://github.com/esp-cpp/uart_serial_plotter)**: For real-time plotting

## Troubleshooting

### Common Issues

**No device found (Device ID read fails):**
- Check I2C wiring (SDA, SCL, power, ground)
- Verify power supply voltage (3.3V)

**Noisy data:**
- Check power supply stability
- Add decoupling capacitors (100nF ceramic + 10µF electrolytic)
- Ensure stable mechanical mounting
- Consider using performance mode for gyroscope
- Adjust filter bandwidth settings

**No data updates:**
- Check if `has_new_data()` returns true
- Verify ODR (Output Data Rate) settings
- Check interrupt configuration if using interrupt mode
- Ensure proper timing in update loop

**Orientation drift:**
- Calibrate gyroscope offset
- Tune filter parameters (process/measurement noise)
- Consider magnetometer integration for yaw correction
- Check for temperature effects

### Debug Tips

1. **Enable verbose logging:**
   ```cpp
   config.log_level = espp::Logger::Verbosity::DEBUG;
   ```

2. **Check register values:**
   ```cpp
   uint8_t status = imu.read_u8_from_register(0x03, ec);  // STATUS register
   fmt::print("Status: 0x{:02X}\n", status);
   ```

3. **Monitor timing:**
   ```cpp
   auto start = esp_timer_get_time();
   imu.update(dt, ec);
   auto elapsed = esp_timer_get_time() - start;
   fmt::print("Update time: {} µs\n", elapsed);
   ```

## Performance Notes

- **Update Rate**: Up to 100Hz in this example (configurable)
- **Latency**: < 1ms for basic IMU reading
- **Memory Usage**: ~6KB stack for IMU task
- **CPU Usage**: < 5% at 100Hz update rate on ESP32-S3
- **Power Consumption**: ~685µA for BMI270 in normal mode

## Advanced Features

The example can be extended to demonstrate:

- **Motion Detection**: Any motion, no motion, significant motion
- **Step Counting**: Pedometer functionality
- **Activity Recognition**: Walk, run, still classification
- **Gesture Detection**: Wrist gestures, tap detection
- **FIFO Usage**: Batch data collection
- **Low Power Modes**: Optimized for battery applications

## References

- [BMI270 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
- [Kalman Filter Theory](https://en.wikipedia.org/wiki/Kalman_filter)
- [Madgwick Filter Paper](https://x-io.co.uk/res/doc/madgwick_internal_report.pdf)
- [ESP-CPP Documentation](https://esp-cpp.github.io/espp/) 
