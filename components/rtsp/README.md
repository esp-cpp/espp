# RTSP (Real-Time Streaming Protocol) Component

[![Badge](https://components.espressif.com/components/espp/rtsp/badge.svg)](https://components.espressif.com/components/espp/rtsp)

The `rtsp` component provides various classes for implementing both sides of an
RTSP stream for transmitting MJPEG video data.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [RTSP (Real-Time Streaming Protocol) Component](#rtsp-real-time-streaming-protocol-component)
  - [RTSP Client](#rtsp-client)
  - [RTSP Server](#rtsp-server)
  - [Testing and Utilities](#testing-and-utilities)
  - [Example](#example)

<!-- markdown-toc end -->

## RTSP Client

The `RtspClient` class provides an interface to an RTSP server. It is used to
send RTSP requests and receive RTSP responses. It also provides an interface
to the RTP and RTCP sessions that are created as a result of the RTSP
interactions.

The `RtspClient` currently only supports MJPEG streams, since the ESP32 does
not have a hardware decoder for H.264 or H.265.

Additionally the client currently only supports UDP transport for RTP and RTCP
packets. TCP transport is not supported.

The user can register a callback function to be notified when new, complete JPEG
frames are received. The callback function is called with a pointer to the JPEG
frame.

## RTSP Server

The `RtspServer` class provides an implementation of an RTSP server. It is used
to receive RTSP requests and send RTSP responses. It is designed to allow the
user to send JPEG frames to the server, which will then send them to the client
over RTP/UDP.

The server currently only supports MJPEG streams, since the ESP32 does not have
a hardware encoder for H.264 or H.265.

Additionally, the server currently only supports UDP transport for RTP and RTCP
packets. TCP transport is not supported.

## Testing and Utilities

We have a few ways for testing the RTSP code:
- **ESPP Python Library**:
  [`espp/lib`](https://github.com/esp-cpp/espp/tree/main/lib) contains the code
  and scripts needed to build the ESPP Python library, which can be used to test
  the RTSP server and client using Python scripts. There are existing RTSP
  python tests within
  [`espp/python`](https://github.com/esp-cpp/espp/tree/main/python) which use
  this lib and provide various RTSP client / server functionality.
- **Pure Python**: The [`rtsp/python`](./python) folder contains pure Python
  scripts that can be used to connect to and display the RTSP stream from an
  RTSP server, using both mDNS and directly by IP address.

### ESPP Python Library

The [`espp/lib`](https://github.com/esp-cpp/espp/tree/main/lib) folder contains
the code and scripts needed to build the ESPP Python library, which can be used
to test the RTSP server and client functionality. This library provides a Python
interface to the RTSP server and client classes, allowing you to easily send and
receive RTSP requests and responses, as well as send and receive JPEG frames
over RTP/UDP.

The [`espp/python`](https://github.com/esp-cpp/espp/tree/main/python) folder
contains the actual python scripts which run espp c++ code (via `pybind11`). See
the `README` in that directory for more information on how to use those scripts.

### Pure Python

To facilitate testing and debugging of the RTSP code, there are some
[python](./python) scripts in this folder which provide various mechanisms for
connecting to and displaying the RTSP stream from an RTSP server, using both
mDNS and directly by IP address. These scripts can be used to test the RTSP
server and client implementations.

See [python/README.md](./python/README.md) for more information on how to use
these scripts.

## Example

The [example](./example) shows the use of the `espp::RtspServer` and
`espp::RtspClient` classes provided by the `rtsp` component for performing
streaming of JPEG images (`espp::JpegFrame`) using the `MJPEG` format over the
Real Time Streaming Protocol (RTSP) / Real Time Protocol (RTP) packets.

For more complete example use, see the
[camera-streamer](https://github.com/esp-cpp/camera-streamer) and
[camera-display](https://github.com/esp-cpp/camera-display) repositories.
