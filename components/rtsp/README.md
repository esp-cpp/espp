# RTSP (Real-Time Streaming Protocol) Component

[![Badge](https://components.espressif.com/components/espp/rtsp/badge.svg)](https://components.espressif.com/components/espp/rtsp)

The `rtsp` component provides various classes for implementing both sides of an
RTSP stream for transmitting MJPEG video data.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [RTSP (Real-Time Streaming Protocol) Component](#rtsp-real-time-streaming-protocol-component)
  - [RTSP Client](#rtsp-client)
  - [RTSP Server](#rtsp-server)
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

## Example

The [example](./example) shows the use of the `espp::RtspServer` and
`espp::RtspClient` classes provided by the `rtsp` component for performing
streaming of JPEG images (`espp::JpegFrame`) using the `MJPEG` format over the
Real Time Streaming Protocol (RTSP) / Real Time Protocol (RTP) packets.

For more complete example use, see the
[camera-streamer](https://github.com/esp-cpp/camera-streamer) and
[camera-display](https://github.com/esp-cpp/camera-display) repositories.
