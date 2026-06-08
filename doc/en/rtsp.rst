RTSP APIs
*********

The ``rtsp`` component provides a flexible, multi-codec RTSP streaming framework
for ESP32 devices. It supports MJPEG, H.264, and generic audio codecs through
an extensible packetizer/depacketizer architecture. The component handles RTP
packet splitting and reassembly; encoding and decoding of media data is handled
externally by the application.

RTSP Client
-----------

The ``RtspClient`` class connects to an RTSP server and receives media streams
over RTP/UDP. It dispatches incoming RTP packets to codec-specific
depacketizers based on payload type.

For backward compatibility, setting the ``on_jpeg_frame`` callback
automatically creates an ``MjpegDepacketizer`` for MJPEG streams (payload type
26). For generic multi-track use, applications can use the ``on_frame`` callback
and inspect parsed SDP metadata through ``tracks()``.

The client now supports:

- generic ``on_frame(track_id, data)`` callbacks for multi-track sessions
- parsed SDP track metadata including media type, payload type, codec name,
  sample rate, channel count, and resolved control path
- automatic depacketizer selection for MJPEG, H.264, and generic payloads
  discovered during ``DESCRIBE``
- an ``on_connection_lost`` callback for reconnect / rediscovery workflows when
  the RTSP control socket or RTP stream disappears after playback starts


RTSP Server
-----------

The ``RtspServer`` class accepts RTSP connections and streams media over
RTP/UDP. It supports multiple tracks, each with its own codec-specific
packetizer, SSRC, and sequence numbering.

For backward compatibility, calling ``send_frame(const JpegFrame&)`` lazily
creates a default MJPEG track. For other codecs, register tracks via
``add_track()`` and send frames with ``send_frame(track_id, data)``.

The server also exposes helpers that are useful for embedded capture loops:

- configurable accept, session-dispatch, and per-session control task stack
  sizes
- ``has_active_sessions()`` to avoid capturing when no client is actively
  playing
- ``get_capture_cooldown()`` and ``get_recommended_capture_period()`` so an
  application can slow capture when RTP backpressure is observed
- a legacy MJPEG ``send_frame(std::span<const uint8_t>)`` path that preserves
  the older wire format for existing MJPEG-only users


RTP Packetizers & Depacketizers
-------------------------------

The packetizer/depacketizer abstraction allows the server and client to support
multiple media codecs without changing the RTSP core. Concrete implementations
are provided for:

- **MJPEG** (``MjpegPacketizer`` / ``MjpegDepacketizer``) — RFC 2435 JPEG over RTP
- **H.264** (``H264Packetizer`` / ``H264Depacketizer``) — RFC 6184 with FU-A fragmentation
- **Generic** (``GenericPacketizer`` / ``GenericDepacketizer``) — MTU chunking
  for audio or other pre-encoded payloads, with frame reconstruction based on
  RTP marker / timestamp boundaries

Custom packetizers can be created by subclassing ``RtpPacketizer`` or
``RtpDepacketizer``.


Testing and Utilities
---------------------

There are several ways to exercise the RTSP stack:

- **ESPP Python library**:
  ``espp/lib`` contains the build scripts and bindings used to expose the RTSP
  client / server classes to Python.
- **Python harness scripts**:
  ``espp/python`` contains wrapper and multitrack scripts for exercising legacy
  MJPEG flows, generic multi-track flows, live microphone audio, and end-to-end
  host validation.
- **Embedded examples and downstream apps**:
  the component example plus repositories such as ``camera-streamer`` and
  ``camera-display`` cover practical server/client integrations.

See ``python/README.md`` in the repository root for more information on the
host-side scripts.


Example
-------

The :doc:`rtsp_example` page demonstrates several RTSP usage patterns selected
via menuconfig, including:

- legacy MJPEG server + client behavior
- server-only MJPEG streaming
- client-only MJPEG reception
- API-level packetizer / depacketizer exercises
- multi-track streaming with MJPEG video plus generic audio

For more complete integrations, see the `camera-streamer
<https://github.com/esp-cpp/camera-streamer>`_ and `camera-display
<https://github.com/esp-cpp/camera-display>`_ repositories.


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
