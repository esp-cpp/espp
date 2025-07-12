# QMI8658 Example

This example shows how to use the `espp::Qmi8658` component to initialize and
communicate with an QMI8658 6-axis IMU.

## How to use example

### Hardware Required

This example is designed to run on a custom board with a QMI8658 IMU.
You will need to configure the I2C pins for your specific hardware.

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


Running with `esp-cpp/uart-serial-plotter`:

