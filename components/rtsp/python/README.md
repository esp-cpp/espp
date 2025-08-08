# Python RTSP Code

This folder contains some test scripts for debugging / testing RTSP servers from
python.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Python RTSP Code](#python-rtsp-code)
  - [Setup](#setup)
  - [Scripts](#scripts)
  - [Utilities](#utilities)

<!-- markdown-toc end -->


## Setup

It's recommended to use a virtual environment for this code. You can create one with:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Scripts

There are a selection of scripts in this folder, but the following two are
likely the most helpful, and have been tested to work with the
[esp-cpp/camera-streamer](https://github.com/esp-cpp/camera-streamer) project
(which uses this RTSP server implementation and publishes its service via mDNS).

These scripts are designed to be run from the command line, and they expect the
RTSP server to be running and accessible on the network. They can be used to
test the RTSP server's functionality, check if it is discoverable via mDNS, and
display the video stream from the server. 

They require that you have setup the python virtual environment and installed
the required dependencies as described above.

- `rtsp_client.py`: A simple RTSP client that connects to an RTSP server and
  prints the response. It expects an IP address and port as command line
  arguments. NOTE: you may need to run this code with `sudo` to allow access to
  the network interfaces, depending on your system configuration.
- `rtsp_client_mdns.py`: An RTSP client that discovers RTSP servers using mDNS
  and connects to them. NOTE: you must run this script with `sudo` to allow mDNS
  to work properly and to be able to connect to the network.
- `opencv_rtsp_client.py`: An RTSP client which uses OpenCV's built-in support
  for RTSP streams. It connects to an RTSP server and displays the video stream
  in a window. This script is useful for quickly testing RTSP servers without
  needing to write custom code. NOTE: you may need to run this code with `sudo`
  to allow access to the network interfaces, depending on your system
  configuration.

## Utilities

- `display_frame.py`: Simple utility to open a binary image (such as something
  saved via a python RTSP client script) and display it via opencv. Can also be
  useful when debugging and ensuring that the RTSP client / server code are
  receiving valid data.
