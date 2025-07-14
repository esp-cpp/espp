# Waveshare ESP32-S3 TouchLCD Example

This example shows how to use the `espp::WsS3Touch` hardware abstraction
component to automatically detect and initialize components on the Waveshare
ESP32-S3 TouchLCD board, and use the LVGL graphics library to draw on the
display. It also plays a sound when you touch the screen using the buzzer on the
system, and it changes the frequency of the sound based on the position of the
touch.

<img width="1260" height="949" alt="image" src="https://github.com/user-attachments/assets/17e8def4-0d64-4e9e-8cee-dc2753a657a6" />
<img width="1260" height="949" alt="image" src="https://github.com/user-attachments/assets/512ca611-1ab0-4941-b4a3-4ef22d65e567" />

https://github.com/user-attachments/assets/e1905efc-d7cf-448a-86e8-a15c3f294f8c

## How to use example

### Hardware Required

This example is designed to run on the Waveshare ESP32-S3 TouchLCD board.

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

<img width="777" height="604" alt="CleanShot 2025-07-13 at 22 01 56" src="https://github.com/user-attachments/assets/02ea3513-7af9-43bb-96e4-10706f31f831" />

<img width="1260" height="949" alt="image" src="https://github.com/user-attachments/assets/17e8def4-0d64-4e9e-8cee-dc2753a657a6" />
<img width="1260" height="949" alt="image" src="https://github.com/user-attachments/assets/512ca611-1ab0-4941-b4a3-4ef22d65e567" />

https://github.com/user-attachments/assets/e1905efc-d7cf-448a-86e8-a15c3f294f8c

