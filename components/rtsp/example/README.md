# RTSP Example

This example demonstrates several `rtsp` component workflows, ranging from the
legacy MJPEG API to the newer multi-track server API. The example can run as a
server, client, or combined self-check depending on the selected menuconfig
mode.

For more complete example use, see the
[camera-streamer](https://github.com/esp-cpp/camera-streamer) and
[camera-display](https://github.com/esp-cpp/camera-display) repositories.

## How to use example

### Hardware Required

Any supported ESP32 board with Wi-Fi can run this example. The bundled test
content uses an embedded JPEG image and synthetic audio data, so no camera,
microphone, or display hardware is required.

### Configure the project

```sh
idf.py menuconfig
```

The `RTSP Example Configuration` menu includes:

* `Example Mode`
  * `Legacy MJPEG (server + client)` - runs the original MJPEG server and
    client on the same device
  * `MJPEG Server only` - serves the embedded JPEG on `/mjpeg/1`
  * `MJPEG Client only` - connects to a remote MJPEG RTSP server
  * `Multi-track server (MJPEG + audio)` - serves MJPEG video on track 0 and
    generic `L16/16000` mono audio on track 1 at `/stream`
* `Run API tests at startup` - exercises packetizers, depacketizers, and basic
  client/server construction before networking starts
* Wi-Fi credentials and retry count
* `RTSP Server Port`
* `Remote RTSP server address` when client-only mode is selected

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```sh
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to
build projects.

## Example Output

The serial log reports the selected mode, the local IP address after Wi-Fi
connects, and the RTSP URI for server modes. Typical output includes:

* `All API tests passed!` when startup API tests are enabled
* `RTSP URI: rtsp://<ip>:8554/mjpeg/1` in legacy and server-only modes
* `RTSP URI: rtsp://<ip>:8554/stream` in multi-track mode
* `Got JPEG frame: <width>x<height>` in client-only mode
* `Streaming MJPEG video (track 0) + audio (track 1)...` in multi-track mode

For host-side end-to-end testing, build the host library in `lib/` with
`./build.sh` and use the scripts in the repository-root `python/` directory.
