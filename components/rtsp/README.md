# RTSP (Real-Time Streaming Protocol) Component

[![Badge](https://components.espressif.com/components/espp/rtsp/badge.svg)](https://components.espressif.com/components/espp/rtsp)

The `rtsp` component provides a flexible, multi-codec RTSP streaming framework
for ESP32 devices. It supports MJPEG, H.264, and generic audio codecs through
an extensible packetizer/depacketizer architecture. The component handles only
RTP packet splitting and reassembly — encoding and decoding of media data is
performed externally.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [RTSP (Real-Time Streaming Protocol) Component](#rtsp-real-time-streaming-protocol-component)
  - [RTSP Client](#rtsp-client)
  - [RTSP Server](#rtsp-server)
  - [Packetizers and Depacketizers](#packetizers-and-depacketizers)
  - [Testing and Utilities](#testing-and-utilities)
  - [Example](#example)

<!-- markdown-toc end -->

## RTSP Client

The `RtspClient` class connects to an RTSP server and receives media streams
over RTP/UDP. It dispatches incoming RTP packets to registered depacketizers
based on payload type.

For **backward compatibility**, setting the `on_jpeg_frame` callback
automatically creates an `MjpegDepacketizer` for MJPEG streams (payload type
26). For other codecs, register a depacketizer via `add_depacketizer()`.

## RTSP Server

The `RtspServer` class accepts RTSP connections and streams media over RTP/UDP.
It supports multiple media tracks, each with its own codec-specific packetizer,
SSRC, and sequence numbering.

For **backward compatibility**, calling `send_frame(const JpegFrame&)` lazily
creates a default MJPEG track. For other codecs, register tracks via
`add_track()` and send frames with `send_frame(track_id, data)`.

## Packetizers and Depacketizers

The packetizer/depacketizer abstraction allows the server and client to support
any media codec without changes to the core RTSP/RTP logic:

- **MJPEG** (`MjpegPacketizer` / `MjpegDepacketizer`) — RFC 2435 JPEG over RTP
- **H.264** (`H264Packetizer` / `H264Depacketizer`) — RFC 6184 with FU-A fragmentation
- **Generic** (`GenericPacketizer` / `GenericDepacketizer`) — Simple MTU chunking for audio

Custom packetizers can be created by subclassing `RtpPacketizer` or
`RtpDepacketizer`.

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
