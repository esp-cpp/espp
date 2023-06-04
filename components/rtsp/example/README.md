# RTSP Example

This example shows the use of the `espp::RtspServer` and `espp::RtspClient`
classes provided by the `rtsp` component for performing streaming of JPEG images
(`espp::JpegFrame`) using the `MJPEG` format over the Real Time Streaming
Protocol (RTSP) / Real Time Protocol (RTP) packets.

For more complete example use, see the
[camera-streamer](https://github.com/esp-cpp/camera-streamer) and
[camera-display](https://github.com/esp-cpp/camera-display) repositories.

## How to use example

### Hardware Required

This example is designed to be run on an M5Stack ESP32 Timer Cam module (server) or a
ESP32-S3-Box (client).

### Configure the project

```
idf.py menuconfig
```

You need to configure the `WiFi` network in the `RTSP Example Configuration`
menuconfig and you can optionally configure the `RTSP Server Port` (default
8554).

### Build and Flash

NOTE: this example is designed to be modified into client only or server-only operation depending on the hardware it is deployed to.

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Example showing the server (camera), the client (handheld display), and the python client, with both clients connected simultaneously:

https://user-images.githubusercontent.com/213467/236601258-c334e1ba-5e18-4452-b48d-e792ec2ed4fb.mp4

Screenshot showing the server running for a very long time (all day, see timestamp in log > 33,000 seconds):
![all_day_test](https://user-images.githubusercontent.com/213467/236601320-0d9139d7-0333-4c63-b26f-da4078e141b7.png)

Screenshot showing the received framerate on the camera-display:
<img width="638" alt="CleanShot 2023-05-06 at 10 27 21@2x" src="https://user-images.githubusercontent.com/213467/236633241-a2aba704-e10f-4855-b07b-766a8f8d8658.png">

Screenshot of main code for camera-streamer output:
<img width="799" alt="CleanShot 2023-05-06 at 10 29 31@2x" src="https://user-images.githubusercontent.com/213467/236633321-abdd2551-0a53-4be2-b90e-a693dfa89c12.png">
