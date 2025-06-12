# Drv2605 Example

This example shows the use of the `drv2605` component to communicate with and
control a DRV2605 I2C haptic motor driver for linear resonant actuator (LRA) and
eccentric rotating mass (ERM) haptic motors.

![CleanShot 2025-06-12 at 11 16 22](https://github.com/user-attachments/assets/cc996d12-fa8e-4c20-8267-b3f1b912c1fe)

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Drv2605 Example](#drv2605-example)
  - [How to use example](#how-to-use-example)
    - [Hardware Required](#hardware-required)
    - [Build and Flash](#build-and-flash)
    - [Output](#output)

<!-- markdown-toc end -->

## How to use example

### Hardware Required

This example can be used with the [Adafruit DRV2605
Breakout](https://www.adafruit.com/product/2305) over I2C.

The sample has code for both ERM and LRA, but make sure to update the call to
`drv2605.select_library(...)` depending on which type of motor you have (ERM is
1-5, LRA is 6).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Output

![CleanShot 2025-06-12 at 11 16 22](https://github.com/user-attachments/assets/cc996d12-fa8e-4c20-8267-b3f1b912c1fe)

older:

![image](https://user-images.githubusercontent.com/213467/225453151-eeba4c4f-7070-4e87-9aa6-741f8a2400ca.png)

