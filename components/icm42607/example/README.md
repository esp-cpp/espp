# ICM42607 Example

This example shows how to use the `espp::Icm42607` component to initialize and
communicate with an ICM42607 / ICM42670 6-axis IMU.

![CleanShot 2025-03-04 at 15 14 10](https://github.com/user-attachments/assets/131e102c-bcab-4732-8593-aa0c56aaebd5)

## How to use example

### Hardware Required

This example is designed to run on the ESP32-S3-BOX or ESP32-S3-BOX-3, as they
have on-board IMUs.

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

![CleanShot 2025-03-04 at 15 15 45](https://github.com/user-attachments/assets/ac2db65c-8be5-4a31-820c-2f7e88cc8ea5)

Running with `esp-cpp/uart-serial-plotter`:
![CleanShot 2025-03-04 at 15 14 10](https://github.com/user-attachments/assets/131e102c-bcab-4732-8593-aa0c56aaebd5)
