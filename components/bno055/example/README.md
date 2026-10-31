# BNO055 Example

This example shows how to use the `espp::Bno055` component to initialize a
BNO055 9-axis absolute orientation IMU over I2C and print its outputs.

It puts the BNO055 in IMU fusion mode (accelerometer + gyroscope, heading
relative to power-on), reads all outputs at 100 Hz in one I2C burst per sample,
and prints one CSV row per sample: acceleration, angular rate, Euler angles, the
orientation quaternion, temperature, and calibration status. The header line
starts with `%` so the output can be pasted straight into a plotter.

## How to use example

### Hardware Required

This example runs on any ESP32 series device with a BNO055 breakout connected
over I2C. The defaults are SDA=GPIO7 and SCL=GPIO8 on the ESP32-P4 (the
internal I2C bus pins on the espp `esp32-p4-eth` board) and SDA=GPIO21,
SCL=GPIO22 on other targets. Change the pins and the I2C address (0x28 or 0x29)
with `idf.py menuconfig` under `Example Configuration`.

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

```
% Time (s), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), Heading (rad), Roll (rad), Pitch (rad), Quat W, Quat X, Quat Y, Quat Z, Temp (C), Calib Sys, Calib Gyro, Calib Accel, Calib Mag
```
