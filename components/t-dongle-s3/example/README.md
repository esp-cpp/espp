# T-Dongle-S3 Example

This example shows how to use the `espp::TDongleS3` hardware abstraction component
initialize the components on the LilyGo T-Dongle-S3.

It initializes the touch, display, and t-keyboard subsystems. It reads the
touchpad state and each time you touch the screen it uses LVGL to draw a circle
where you touch. If you press the home button on the display, it will clear the
circles.

https://github.com/esp-cpp/espp/assets/213467/15f57640-d3bc-4a40-932c-d9309337301f

## How to use example

### Hardware Required

This example is designed to run on the [LilyGo T-Dongle
S3](https://github.com/Xinyuan-LilyGO/T-Dongle-S3) dev board which has:

* WiFi / BLE
* uSD card (hidden in the USB A connector!)
* RGB LED
* Color TFT LCD (ST7735, 80x160 0.96" IPS LCD)

##### T-Dongle S3 Pin Configuration

| LED Pin | ESP32S3 IO Pin Number |
|---------|-----------------------|
| Data    | 40                    |
| Clock   | 39                    |

| LCD Pin   | ESP32S3 IO Pin Number        |
|-----------|------------------------------|
| CS        | 4                            |
| SDA       | 3                            |
| SCL       | 5                            |
| DC        | 2                            |
| Reset     | 1                            |
| Backlight | 38                           |

| TF / uSD Card Pin | ESP32S3 IO Pin Number |
|-------------------|-----------------------|
| D0                | 14                    |
| D1                | 17                    |
| D2                | 21                    |
| D3                | 18                    |
| CLK               | 12                    |
| CMD               | 16                    |


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

![CleanShot 2024-07-06 at 20 12 35@2x](https://github.com/esp-cpp/espp/assets/213467/36a796ae-af6e-4db6-a4f0-628fb49e52cd)

![image](https://github.com/esp-cpp/espp/assets/213467/b869d844-41f9-40da-9967-e9f1c10d7501)
