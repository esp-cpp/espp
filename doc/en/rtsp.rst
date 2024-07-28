RTSP APIs
*********

RTSP Client
-----------

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


RTSP Server
-----------

The `RtspServer` class provides an implementation of an RTSP server. It is used
to receive RTSP requests and send RTSP responses. It is designed to allow the
user to send JPEG frames to the server, which will then send them to the client
over RTP/UDP.

The server currently only supports MJPEG streams, since the ESP32 does not have
a hardware encoder for H.264 or H.265.

Additionally, the server currently only supports UDP transport for RTP and RTCP
packets. TCP transport is not supported.

.. ------------------------------- Example -------------------------------------

.. toctree::

   rtsp_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/rtsp_client.inc
.. include-build-file:: inc/rtsp_server.inc
.. include-build-file:: inc/rtsp_session.inc
.. include-build-file:: inc/rtp_packet.inc
.. include-build-file:: inc/rtp_jpeg_packet.inc
.. include-build-file:: inc/rtcp_packet.inc
.. include-build-file:: inc/jpeg_header.inc
.. include-build-file:: inc/jpeg_frame.inc
