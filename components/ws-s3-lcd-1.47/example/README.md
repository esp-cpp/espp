# Waveshare ESP32-S3-LCD-1.47 Example

This example shows how to use the `espp::WsS3Lcd147` hardware abstraction
component initialize the components on the Waveshare ESP32-S3-LCD-1.47.

It initializes the display, button, and RGB LED subsystems.

## How to use example

### Hardware Required

This example is designed to run on the [Waveshare
ESP32-S3-LCD-1.47](https://www.waveshare.com/wiki/ESP32-S3-LCD-1.47) dev board
which has:

* WiFi / BLE
* uSD card
* Color TFT LCD (ST7789, 172x320 1.47" IPS LCD)
* Boot Button
* RGB LED

##### Waveshare ESP32-S3-LCD-1.47 Pin Configuration

| LCD Pin   | ESP32S3 IO Pin Number |
|-----------|-----------------------|
| MOSI      | 45                    |
| SCLK      | 40                    |
| CS        | 42                    |
| DC        | 41                    |
| Reset     | 39                    |
| Backlight | 48                    |

| TF / uSD Card Pin | ESP32S3 IO Pin Number |
|-------------------|-----------------------|
| D0                | 16                    |
| D1                | 18                    |
| D2                | 17                    |
| D3                | 21                    |
| CLK               | 14                    |
| CMD               | 15                    |

| Misc    | ESP32S3 IO Pin Number |
|---------|-----------------------|
| RGB LED | 38                    |


### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Example Output

```console
I (350) main_task: Calling app_main()
[WsS3Lcd147 Example/I][0.026]: Starting example!
[WsS3Lcd147/I][0.032]: Initializing LCD...
I (364) gpio: GPIO[39]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (372) gpio: GPIO[41]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
[WsS3Lcd147/I][0.454]: Initializing display with pixel buffer size: 16000 bytes
W (782) ledc: the binded timer can't keep alive in sleep
[WsS3Lcd147/I][0.459]: Initializing SD card
I (788) gpio: GPIO[14]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (797) gpio: GPIO[15]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (805) gpio: GPIO[16]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (813) gpio: GPIO[18]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (822) gpio: GPIO[17]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
I (830) gpio: GPIO[21]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (880) gpio: GPIO[21]| InputEn: 0| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0
[WsS3Lcd147/I][0.561]: Filesystem mounted
Name: 00000
Type: SDHC
Speed: 40.00 MHz (limit: 40.00 MHz)
Size: 30543MB
CSD: ver=2, sector_size=512, capacity=62552064 read_bl_len=9
SSR: bus_width=4
[WsS3Lcd147/I][0.568]: Initializing LED
[WsS3Lcd147 Example/I][0.573]: Initializing the button
[WsS3Lcd147/I][0.578]: Initializing button
I (910) gpio: GPIO[0]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:3
Setting rotation to 1
Setting rotation to 2
Setting rotation to 3
Setting rotation to 0
```
