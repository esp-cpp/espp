# Matouch-Rotary-Display Example

This example shows how to use the `espp::MatouchRotaryDisplay` hardware
abstraction component initialize the components on the [MaTouch Rotary Display](https://wiki.makerfabs.com/MaTouch_ESP32_S3_Rotary_IPS_Display_1.28_GC9A01.html).

It initializes the touch, display, and t-keyboard subsystems. It reads the
touchpad state and each time you touch the screen it uses LVGL to draw a circle
where you touch. If you press the home button on the display, it will clear the
circles.

![image](https://github.com/esp-cpp/espp/assets/213467/2d8bf057-5c13-46ee-bf48-54df4ba74aea)

![image](https://github.com/esp-cpp/espp/assets/213467/81f03ddf-0c3e-4db8-8a86-f8749245bd56)

![image](https://github.com/esp-cpp/espp/assets/213467/516731e9-ad5d-487b-a46a-d5b760e66d60)

## How to use example

### Hardware Required

This example is designed to run on the MaTouch Rotary Display 1.28".

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

![CleanShot 2024-07-08 at 22 02 08](https://github.com/esp-cpp/espp/assets/213467/3ab6b27c-9182-4ec5-be95-f85ecfcfea7b)

