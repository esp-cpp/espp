RTSP APIs
*********

The ``rtsp`` component provides a flexible, multi-codec RTSP streaming framework
for ESP32 devices. It supports MJPEG, H.264, and generic audio codecs through
an extensible packetizer/depacketizer architecture. The component handles RTP
packet splitting and reassembly; encoding and decoding of media data is handled
externally by the application.

How RTSP Works
--------------

The component uses a split control-plane / media-plane design:

- **RTSP over TCP** handles session control such as ``OPTIONS``, ``DESCRIBE``,
  ``SETUP``, ``PLAY``, ``PAUSE``, and ``TEARDOWN``.
- **SDP** returned from ``DESCRIBE`` tells the client what tracks exist, how
  they are encoded, and which per-track control URLs must be used for
  ``SETUP``.
- **RTP/UDP** carries encoded media packets after playback starts.
- **RTCP/UDP** sockets are created alongside RTP sockets, but the current ESPP
  implementation keeps RTCP support lightweight and does not yet implement a
  full control/feedback plane.

.. mermaid::

   sequenceDiagram
     participant App as Application
     participant Server as RtspServer / RtspSession
     participant Client as RtspClient
     App->>Server: add_track() / send_frame()
     Client->>Server: OPTIONS
     Server-->>Client: 200 OK
     Client->>Server: DESCRIBE
     Server-->>Client: SDP with session + track control paths
     Client->>Server: SETUP(trackID=n, client_port=RTP-RTCP)
     Server-->>Client: Session + Transport headers
     Client->>Server: PLAY
     Server-->>Client: 200 OK
     Server-->>Client: RTP/UDP packets for each active track
     Client-->>App: on_jpeg_frame() or on_frame(track_id, data)
     Client->>Server: TEARDOWN
     Server-->>Client: 200 OK

In ESPP, the server generates one SDP description per session, with one
``m=...`` section and one ``a=control:.../trackID=N`` entry per registered
track. The client parses those lines during ``describe()`` and then issues
``SETUP`` once per discovered track before calling ``PLAY``.

Packetization Pipeline
----------------------

The codec-specific logic is intentionally separated from the RTSP core:

.. mermaid::

   flowchart LR
     Frame["Encoded frame bytes"] --> Packetizer["Codec packetizer"]
     Packetizer --> Chunks["RTP payload chunks"]
     Chunks --> Header["RtspServer adds RTP headers"]
     Header --> Session["RtspSession sends UDP packets"]
     Session --> ClientRtp["RtspClient RTP socket"]
     ClientRtp --> Depacketizer["Codec depacketizer"]
     Depacketizer --> Callback["Application callback"]

``RtspServer::send_frame(track_id, data)`` asks the selected packetizer to split
the encoded frame into MTU-sized chunks, adds RTP headers with track-specific
SSRC and sequence numbers, and leaves the resulting packets queued for active
sessions to transmit. On the client side, ``RtspClient::handle_rtp_packet()``
parses the RTP header, uses the payload type to find the matching depacketizer,
and emits a completed frame through either ``on_jpeg_frame`` or the generic
``on_frame(track_id, data)`` callback.

Legacy MJPEG Compatibility
--------------------------

For backward compatibility, the component still preserves the older MJPEG-only
behavior:

- ``RtspServer::send_frame(std::span<const uint8_t>)`` lazily creates a default
  track 0 and uses the legacy RFC 2435-compatible MJPEG wire format.
- ``RtspClient`` automatically creates an ``MjpegDepacketizer`` when a JPEG
  callback is registered and payload type 26 is discovered in SDP.

This means older single-track MJPEG integrations can keep working while newer
multi-track applications use ``add_track()`` plus codec-specific packetizers.

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

Relevant Specifications
-----------------------

These are the main standards to keep beside the code when working on this
component:

.. list-table::
   :header-rows: 1

   * - Specification
     - Why it matters here
   * - `RFC 2326: Real Time Streaming Protocol (RTSP) <https://datatracker.ietf.org/doc/html/rfc2326>`_
     - Primary control-plane reference for the RTSP/1.0 request and response
       flow implemented by ``RtspClient``, ``RtspServer``, and ``RtspSession``.
   * - `RFC 7826: RTSP 2.0 <https://datatracker.ietf.org/doc/html/rfc7826>`_
     - Useful background for newer RTSP deployments; informative here because
       the current component speaks RTSP/1.0 on the wire.
   * - `RFC 3550: RTP / RTCP <https://datatracker.ietf.org/doc/html/rfc3550>`_
     - Defines RTP headers, timestamps, sequence numbers, SSRC handling, and
       the RTCP control protocol model used by the transport layer.
   * - `RFC 4566: Session Description Protocol (SDP) <https://datatracker.ietf.org/doc/html/rfc4566>`_
     - Describes the SDP ``m=``, ``a=control:``, and ``a=rtpmap:`` lines that
       the server generates and the client parses during ``DESCRIBE``.
   * - `RFC 3551: RTP A/V Profile <https://datatracker.ietf.org/doc/html/rfc3551>`_
     - Defines common RTP payload-type and clock-rate conventions used alongside
       dynamic payloads.
   * - `RFC 2435: RTP Payload Format for JPEG <https://datatracker.ietf.org/doc/html/rfc2435>`_
     - Reference for the MJPEG packetization and depacketization path.
   * - `RFC 6184: RTP Payload Format for H.264 Video <https://datatracker.ietf.org/doc/html/rfc6184>`_
     - Reference for the H.264 FU-A fragmentation and reassembly path.


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
