# ESPP Python Library

This folder contains some test python scripts for loading the `espp` shared
objects built within the `espp/lib` folder into Python.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESPP Python Library](#espp-python-library)
  - [Scripts](#scripts)
  - [Setup](#setup)
    - [Install Python Requirements](#install-python-requirements)
  - [Quick Start](#quick-start)

<!-- markdown-toc end -->

## Scripts

This section gives a brief overview of what the scripts in this folder do.

- `task.py`: This script is a simple example of how to use the `espp` library to
  create a task. It demonstrates how to create a task, run it, and handle its
  results. Note: unlike python multithreading, this task is actually run in a
  separate thread.
- `timer.py`: This script demonstrates how to use the `espp` library to create a
  timer. It shows how to create a timer, start it, and handle its expiration.
  The timer runs in a separate thread and calls a callback function when it
  expires. Note: unlike python multithreading, this timer is actually run in a
  separate thread.
- `udp_client.py` and `udp_server.py`: These scripts demonstrate how to use the
  `espp` library to create a UDP client and server. The server listens for
  incoming UDP packets and prints them to the console, while the client sends
  UDP packets to the server.
- `rtsp_client.py` and `rtsp_server.py`: These scripts demonstrate how to use the
  `espp` library to create an RTSP client and server. The server streams MJPEG
  video from a webcam or display capture. The camera path captures live
  microphone audio by default, while display capture keeps the simulated audio
  tone unless `--audio-source microphone` is selected. The client receives the
  stream, validates audio delivery, plays the audio in real time, and displays
  the video in a window. Note: these scripts require additional 3rd party
  libraries such as `opencv`, `mss`, `zeroconf`, and `sounddevice`.
- `rtsp_client_multitrack.py` and `rtsp_server_multitrack.py`: These scripts
  exercise the generic multitrack RTSP APIs. The multitrack server includes
  audio by default; camera mode uses live microphone capture and the other video
  sources use the simulated tone unless `--audio-source microphone` is selected.
  Pass `--no-audio` to disable the audio track. The multitrack client plays
  audio by default when running with a UI; use `--no-audio-playback` to disable
  it or `--play-audio --headless` to exercise playback without opening the video
  window.
- `rtps_host.py`: A pure-stdlib host-side RTPS harness for discovering an ESPP
  `RtpsParticipant`, printing SPDP/SEDP metadata, and optionally publishing or
  receiving standard CDR-over-RTPS `UInt32` user-data samples (routed by writer
  GUID via SEDP) without needing Python bindings. It follows endpoint-advertised
  user-data multicast locators, joining matching subscribed-topic multicast
  groups dynamically. Run `python rtps_host.py --self-test` to validate the
  wire-format encoders/decoders against the firmware with no network I/O.
- `cobs_demo.py`: Demonstration of ESPP COBS functionality with native Python data types.
  Shows ESPP encoding/decoding, cross-library compatibility with the cobs-python library,
  and practical usage examples. Includes design differences explanation and validation
  against the reference implementation.

**Important Design Differences**: The ESPP COBS implementation differs from other COBS libraries:
- **Delimiters**: ESPP automatically adds `0x00` delimiters to encoded packets, while other libraries may not
- **Empty Packets**: ESPP ignores empty packets (length = 0) for performance, while other libraries may encode them
- **Compatibility**: The demo scripts show how to handle these differences for cross-library integration
  

## Setup

For all scripts, you must have the `espp` shared objects built and the required Python dependencies installed:

```bash
pip install -r requirements.txt
```

**Note**: The COBS demo script (`cobs_demo.py`) requires the `cobs` library for cross-validation with the reference implementation.
accessible in the `espp/lib` folder. See the [espp/lib](../lib/README.md)
README for more details.

### Install Python Requirements

Some tests (e.g. `rtsp_client.py`, `rtsp_server.py`) make use of 3rd party
libraries such as `zeroconf, opencv, mss, sounddevice` to facilitate mDNS
discovery, image display / webcam capture, display capture, and audio
playback. For these tests, you will need to install the `requirements.txt`:

```console
# create the virtual environment
python3 -m venv env
# activate it
source env/bin/activate
# install the requirements
pip install -r requirements.txt
```

Afterwards, you only need to source the environment and run the test in
question:

```console
source env/bin/activate
# now you can run the tests
python3 rtsp_server.py
```

## Quick Start

To run a test, you can use the following commands:

```console
python3 <test_name>.py
# e.g.
python3 task.py
# or 
python3 udp_client.py
# or discover / test RTPS from a host machine
python3 rtps_host.py --advertised-address <your-host-ip>
```

Note: the `udp_client.py` script requires a running instance of the
`udp_server.py` script. To run the server, use the following command from
another terminal:

```console
python3 udp_server.py
```

For the default ESP RTPS example, the host harness now defaults to the
**responder** side of the request/response test, so it will subscribe to
``espp/rtps_example/request`` and echo values back on
``espp/rtps_example/response``:

```console
python3 rtps_host.py --advertised-address 192.168.1.50
```

When the ESP RTPS example is configured for per-topic multicast, the host
harness will automatically join the discovered request-topic multicast group and
send response samples using the discovered response-reader locators.

To act as the initiator instead, swap the topics and enable periodic publishing:

```console
python3 rtps_host.py --advertised-address 192.168.1.50 \
  --subscribe-topic espp/rtps_example/response \
  --publish-topic espp/rtps_example/request \
  --publish-value 42 \
  --publish-interval 1.0 \
  --no-echo-received
```
