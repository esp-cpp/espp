RTSP APIs
*********

The ``rtsp`` component provides a flexible, multi-codec RTSP streaming framework
for ESP32 devices. It supports MJPEG, H.264, and generic audio codecs through
an extensible packetizer/depacketizer architecture. The component handles only
RTP packet splitting and reassembly — encoding and decoding of media data is
performed externally.

RTSP Client
-----------

The ``RtspClient`` class connects to an RTSP server and receives media streams
over RTP/UDP. It dispatches incoming RTP packets to registered depacketizers
based on payload type.

For **backward compatibility**, setting the ``on_jpeg_frame`` callback
automatically creates an ``MjpegDepacketizer`` for MJPEG streams (payload type
26). For other codecs, register a depacketizer via ``add_depacketizer()``.


RTSP Server
-----------

The ``RtspServer`` class accepts RTSP connections and streams media over
RTP/UDP. It supports multiple tracks, each with its own codec-specific
packetizer, SSRC, and sequence numbering.

For **backward compatibility**, calling ``send_frame(const JpegFrame&)`` lazily
creates a default MJPEG track. For other codecs, register tracks via
``add_track()`` and send frames with ``send_frame(track_id, data)``.


RTP Packetizers & Depacketizers
-------------------------------

The packetizer/depacketizer abstraction allows the server and client to support
any media codec. Concrete implementations are provided for:

- **MJPEG** (``MjpegPacketizer`` / ``MjpegDepacketizer``) — RFC 2435
- **H.264** (``H264Packetizer`` / ``H264Depacketizer``) — RFC 6184, FU-A fragmentation
- **Generic** (``GenericPacketizer`` / ``GenericDepacketizer``) — Simple MTU chunking for audio

Custom packetizers can be created by subclassing ``RtpPacketizer`` or
``RtpDepacketizer``.


.. ------------------------------- Example -------------------------------------

.. toctree::

   rtsp_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/rtsp_client.inc
.. include-build-file:: inc/rtsp_server.inc
.. include-build-file:: inc/rtsp_session.inc
.. include-build-file:: inc/rtp_packetizer.inc
.. include-build-file:: inc/rtp_depacketizer.inc
.. include-build-file:: inc/rtp_types.inc
.. include-build-file:: inc/mjpeg_packetizer.inc
.. include-build-file:: inc/mjpeg_depacketizer.inc
.. include-build-file:: inc/h264_packetizer.inc
.. include-build-file:: inc/h264_depacketizer.inc
.. include-build-file:: inc/generic_packetizer.inc
.. include-build-file:: inc/generic_depacketizer.inc
.. include-build-file:: inc/rtp_packet.inc
.. include-build-file:: inc/rtp_jpeg_packet.inc
.. include-build-file:: inc/rtcp_packet.inc
.. include-build-file:: inc/jpeg_header.inc
.. include-build-file:: inc/jpeg_frame.inc
