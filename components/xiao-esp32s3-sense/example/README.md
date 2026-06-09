# XIAO ESP32S3 Sense Example

This example turns the Seeed Studio XIAO ESP32S3 Sense into a small RTSP
camera streamer.

It mirrors the newer ESP32-TimerCam streamer-style example:

- connects to Wi-Fi
- breathes the onboard user LED quickly while disconnected, then more slowly once connected
- starts an RTSP server and advertises it over mDNS
- captures MJPEG video from the onboard OV2640 camera
- captures 16 kHz mono PCM audio from the onboard PDM microphone
- mounts the onboard microSD card and performs a simple file write/read smoke test when present
- serves both tracks from a single RTSP session
- exposes a simple serial CLI with Wi-Fi and memory-monitor commands

## How to use example

### Hardware Required

This example is designed to run on the Seeed Studio XIAO ESP32S3 Sense.

### Configuration

Set the Wi-Fi credentials and RTSP port in `menuconfig`:

```bash
idf.py menuconfig
```

Open **Camera Streamer Configuration** and set:

- **WiFi SSID**
- **WiFi Password**
- **RTSP Server Port**
- **RTSP accept/session/control task stack sizes** as needed for your target

The defaults are tuned for the XIAO ESP32S3 Sense, including a larger RTSP
control-task stack to avoid stack overflows in tasks named like
`RtspSession <id>`.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)

After the board joins Wi-Fi it advertises an `_rtsp._tcp` mDNS service and
starts streaming at:

```text
rtsp://<board-ip>:<port>/mjpeg/1
```

Track 0 is MJPEG video and track 1 is L16/16000 mono audio.
