# ICM20948 Example

This example shows how to use the `espp::Icm20948` component to initialize and
communicate with an ICM20948 9-axis IMU.

![CleanShot 2025-03-18 at 12 08 23](https://github.com/user-attachments/assets/2dc0020e-9a11-4f52-ba69-79cfbb91ba02)

## How to use example

### Hardware Required

This example is designed to run on a QtPy Pico or QtPy ESP32-S3 with an ICM20948
qwiic breakout board connected over I2C.

![CleanShot 2025-03-18 at 12 27 13](https://github.com/user-attachments/assets/0531a03d-bc92-4e7a-a3a3-b48cc4405e7e)

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

![CleanShot 2025-03-18 at 12 09 04](https://github.com/user-attachments/assets/557abc88-c034-4833-97cd-c5a5ac43e1e0)

[icm20948-test.txt](https://github.com/user-attachments/files/19323977/icm20948-test.txt)

![CleanShot 2025-03-18 at 12 08 23](https://github.com/user-attachments/assets/2dc0020e-9a11-4f52-ba69-79cfbb91ba02)


