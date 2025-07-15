# Waveshare ESP32-S3-GEEK Example

This example shows how to use the `espp::WsS3Geek` hardware abstraction
component initialize the components on the Waveshare ESP32-S3-GEEK.

It initializes the display, and button subsystems.

## How to use example

### Hardware Required

This example is designed to run on the [Waveshare
ESP32-S3-GEEK](https://www.waveshare.com/wiki/ESP32-S3-GEEK) dev board which
has:

* WiFi / BLE
* uSD card
* Color TFT LCD (ST7789P3, 240x135 1.14" IPS LCD)
* Boot Button

##### Waveshare ESP32-S3-GEEK Pin Configuration

| LCD Pin    | ESP32S3 IO Pin Number |
|------------|-----------------------|
| CS         | 10                    |
| SDA (MOSI) | 11                    |
| SCL (CLK)  | 12                    |
| DC         | 8                     |
| Reset      | 9                     |
| Backlight  | 7                     |

| TF / uSD Card Pin | ESP32S3 IO Pin Number |
|-------------------|-----------------------|
| D0 / MISO         | 37                    |
| D1                | 33                    |
| D2                | 38                    |
| D3 / CS           | 34                    |
| CLK               | 36                    |
| CMD / MOSI        | 35                    |


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


```console
[WsS3Geek Example/I][0.026]: Starting example!
[WsS3Geek/I][0.032]: Initializing LCD...
I (357) gpio: GPIO[8]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (365) gpio: GPIO[9]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
[WsS3Geek/I][0.453]: Initializing display with pixel buffer size: 12000 bytes
W (776) ledc: the binded timer can't keep alive in sleep
[WsS3Geek/I][0.458]: Initializing SD card
I (782) gpio: GPIO[34]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (830) sdspi_transaction: cmd=52, R1 response: command not supported
I (872) sdspi_transaction: cmd=5, R1 response: command not supported
[WsS3Geek/I][0.576]: Filesystem mounted
Name: 00000
Type: SDHC
Speed: 20.00 MHz (limit: 20.00 MHz)
Size: 30543MB
CSD: ver=2, sector_size=512, capacity=62552064 read_bl_len=9
SSR: bus_width=1
[WsS3Geek Example/I][0.583]: Initializing the button
[WsS3Geek/I][0.589]: Initializing button
I (915) gpio: GPIO[0]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:3
```
