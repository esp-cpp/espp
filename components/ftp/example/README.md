# FTP Example

This example showcases the use of the `FtpServer` from the `ftp` component.

## How to use example

### Configure the project

```
idf.py menuconfig
```

You'll need to configure the `WiFi` information that you wish the ESP to connect
to. You can optionally configure the FTP server port if you wish to use a
non-standard port.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![output](https://user-images.githubusercontent.com/213467/235329930-15755d54-076a-4d4e-bf6f-93f1cbb5196b.png)
