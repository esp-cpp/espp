RTPS APIs
*********

The ``rtps`` component is a cross-platform **RTPS / DDS** stack: it integrates the
`embeddedRTPS <https://github.com/embedded-software-laboratory/embeddedRTPS>`_
engine behind an idiomatic ``espp::RtpsParticipant`` facade so that any platform
that can build ESPP — ESP32, Linux, macOS, Windows — can interoperate with **ROS 2**
nodes (``rmw_fastrtps``) or any DDS participant on the network over the standard
RTPS wire protocol.

It provides three messaging patterns, each validated against live ROS 2 (Jazzy),
and each with a **typed** layer (reflectable structs, no manual bytes) and a
**byte-level** layer:

- **Pub/sub** — topic-based, best-effort or reliable (``HEARTBEAT`` / ``ACKNACK``).
- **Services (RMI)** — request/reply with correlated responses.
- **Actions (AMI)** — long-running goals with feedback, result, and cancellation.

Services and actions come in a ROS 2-interoperable flavour and a lean **native**
(espp ↔ espp) flavour. The wire-format details of the RMI/AMI layers are in
:doc:`rtps_rmi_ami`.

.. note::

   The upstream embeddedRTPS library hard-depends on FreeRTOS and lwIP. This
   component removes those, routing all socket, task, and synchronisation through
   ESPP's platform-agnostic ``UdpSocket``, ``Task``, ``ThreadPool``, and
   ``SocketReactor`` (lwIP + FreeRTOS on ESP32, the host OS elsewhere). Micro-CDR
   is gone — (de)serialization uses ESPP's reflection-driven ``cdr``.

Architecture
------------

The only platform-specific code is ``EsppTransport``; everything above it is
portable C++23. The ``espp::`` facade is a thin, typed surface over the ``rtps::``
engine.

.. mermaid::

   flowchart TD
     U["User code / ROS 2 peer"]

     subgraph facade["espp:: facade (typed + byte-level)"]
       PS["Publisher / Subscriber (typed)"]
       SVC["ServiceServer / ServiceClient"]
       ACT["ActionServer / ActionClient"]
       RP["espp::RtpsParticipant"]
       PS --> RP
       SVC --> RP
       ACT --> RP
     end

     subgraph engine["rtps:: engine (embeddedRTPS, de-vendored)"]
       DOM["rtps::Domain — packet routing + discovery"]
       PART["rtps::Participant"]
       WR["rtps::Writer (history + HEARTBEAT)"]
       RD["rtps::Reader (ACKNACK + delivery)"]
       DISC["SPDP + SEDP discovery agents"]
       DOM --> PART --> WR & RD
       DOM --> DISC
     end

     subgraph plat["platform adapter (the ONLY porting layer)"]
       TR["rtps::EsppTransport"]
       SOCK["espp::UdpSocket × N ports"]
       REACT["espp::SocketReactor → espp::ThreadPool"]
       CDR["espp::cdr (reflection CDR/XCDR)"]
       TR --> SOCK --> REACT
     end

     U --> facade --> engine --> plat
     WR -. serialize .-> CDR
     RD -. deserialize .-> CDR

Discovery Flow
--------------

RTPS separates *metatraffic* (discovery) from *user traffic* (samples). A
participant announces itself with **SPDP** over multicast, then exchanges
**SEDP** endpoint metadata (topic, type, reliability, locators) with each peer.
Once a local writer and a remote reader (or vice-versa) match on topic + type,
user data flows.

.. mermaid::

   sequenceDiagram
     participant A as espp participant
     participant MC as 239.255.0.1 (SPDP)
     participant B as ROS 2 / DDS peer
     A->>MC: SPDP DATA(GUID, locators, builtin endpoints, user_data)
     MC-->>B: multicast delivery
     B->>MC: SPDP DATA(its participant metadata)
     MC-->>A: multicast delivery
     A->>B: SEDP publication/subscription DATA(topic, type, QoS, unicast locator)
     B->>A: SEDP publication/subscription DATA
     Note over A,B: writer/reader match on topic + type → user data flows

Pub/Sub Reliability
-------------------

A **best-effort** writer simply sends ``DATA`` submessages; lost samples are not
recovered. A **reliable** writer keeps a history and periodically piggybacks a
``HEARTBEAT`` advertising its sequence-number range; the reader ``ACKNACK``\ s
what it is missing, and the writer retransmits. This is the state machine that
makes the component interoperate with reliable ROS 2 QoS.

.. mermaid::

   sequenceDiagram
     participant W as Reliable Writer
     participant R as Reader
     W->>R: DATA (seq 1)
     W-xR: DATA (seq 2)
     W->>R: DATA (seq 3)
     Note over W,R: seq 2 was lost
     W->>R: HEARTBEAT (first=1, last=3)
     R->>W: ACKNACK (missing = 2)
     W->>R: DATA (seq 2) retransmit
     R->>W: ACKNACK (all received)

Large samples (> ~64 KB) are split into ``DATA_FRAG`` submessages when
``RTPS_ENABLE_FRAGMENTATION`` is on (default on host, Kconfig opt-in on ESP32);
this interoperates with FastDDS both directions.

Services (RMI)
--------------

A service is two topics — a request and a reply — plus **correlation**. The
client tags its request with a ``related_sample_identity`` inline QoS carrying
its own reply-reader GUID; the server echoes ``{that GUID, the request's sequence
number}`` on the reply, and the client matches the reply back to the pending
call. This is exactly what ``rmw_fastrtps`` does, so an espp service appears in
``ros2 service list`` and answers ``ros2 service call``.

.. mermaid::

   sequenceDiagram
     participant C as ServiceClient
     participant S as ServiceServer
     C->>S: request DATA (rq topic) + related_sample_identity{reply-reader GUID, seq=UNKNOWN}
     Note over S: handler(request) produces response
     S->>C: reply DATA (rr topic) + related_sample_identity{that GUID, request seq}
     Note over C: match on {own GUID, pending seq} → deliver reply

Clients offer three call styles: blocking ``call()``, callback ``call_async()``,
and ``call_future()`` returning a ``std::future``.

Actions (AMI)
-------------

An action adds no new wire primitive: it composes **three services**
(``send_goal`` / ``cancel_goal`` / ``get_result``) and **two topics**
(``feedback`` / ``status``) over the RMI + pub/sub layers. A goal moves through a
small lifecycle the server drives and the client observes:

.. mermaid::

   stateDiagram-v2
     [*] --> ACCEPTED: send_goal (accepted)
     [*] --> REJECTED: send_goal (rejected)
     ACCEPTED --> EXECUTING: execute() starts
     EXECUTING --> EXECUTING: publish_feedback()
     EXECUTING --> SUCCEEDED: succeed(result)
     EXECUTING --> ABORTED: abort(result)
     EXECUTING --> CANCELED: cancel_goal + is_canceling() → canceled(result)
     SUCCEEDED --> [*]: get_result
     ABORTED --> [*]: get_result
     CANCELED --> [*]: get_result
     REJECTED --> [*]

Native protocol
---------------

For espp ↔ espp links that do not need ROS 2 interop, a lean **native** protocol
trades interop for simplicity: services correlate with a 20-byte in-band header
(no inline QoS, no ``rq``/``rr`` mangling) on ``es_rq`` / ``es_rr`` topics, and a
native action is just two services + one topic (a goal service, a cancel
service, and one feedback topic that also carries the terminal result: 5 topics
/ 5 RTPS endpoints per side, vs the ROS 2 action's three services + two topics
= 8). Same client ergonomics. See
:doc:`rtps_rmi_ami` for the byte layout.

Ports and Channels
------------------

The component follows the standard UDPv4 RTPS port mapping formula:

.. list-table::
   :header-rows: 1

   * - Channel
     - Formula
     - Domain 0, participant 0
   * - Metatraffic multicast
     - ``7400 + 250 * domain + 0``
     - ``7400``
   * - Metatraffic unicast
     - ``7400 + 250 * domain + 10 + 2 * participant``
     - ``7410``
   * - User multicast
     - ``7400 + 250 * domain + 1``
     - ``7401``
   * - User unicast
     - ``7400 + 250 * domain + 11 + 2 * participant``
     - ``7411``

Configuration
-------------

Capacity limits are chosen at build time by a **limits profile** header; storage
policy, fragmentation, and the RPC layer are separate, independent knobs (ESP-IDF
menuconfig under ``RTPS`` on ESP32; ``include/rtps/config.hpp`` defaults on host).

.. list-table::
   :header-rows: 1

   * - Knob
     - Options / default
     - Effect
   * - ``RTPS_LIMITS_PROFILE``
     - ``embedded`` (ESP32 default) / ``host`` (host default) / ``host_large``
     - Compile-time endpoint/history capacity caps. Wire-neutral.
   * - ``RTPS_STORAGE_DYNAMIC``
     - off on ESP32 / on host
     - Static ``std::array`` history (zero-heap, drop-oldest) vs heap-backed
       ``std::deque`` (grows). Orthogonal to the profile.
   * - ``RTPS_ENABLE_FRAGMENTATION``
     - off on ESP32 / on host
     - ``DATA_FRAG`` for samples > ~64 KB (interoperates with FastDDS / ROS 2).
   * - ``RTPS_ENABLE_RPC``
     - on (default)
     - Compile in services + actions (RMI/AMI). Disable to drop that code and its
       threads on a pure-pub/sub device.

Relevant Specifications
-----------------------

.. list-table::
   :header-rows: 1

   * - Specification
     - Why it matters here
   * - `OMG DDSI-RTPS 2.3 <https://www.omg.org/spec/DDSI-RTPS/2.3/PDF>`_
     - Primary wire-level reference for RTPS headers, submessages (``DATA``,
       ``HEARTBEAT``, ``ACKNACK``, ``DATA_FRAG``), SPDP, SEDP, locator encoding,
       GUIDs, and the UDP port mapping used by this component.
   * - `OMG DDS 1.4 <https://www.omg.org/spec/DDS/1.4/PDF>`_
     - The participant / reader / writer / topic / QoS model that RTPS carries.

Example
-------

The :doc:`rtps_example` page shows an ESP32 (esp32-ethernet-kit) node that brings
up a participant over Ethernet and exercises the typed APIs — a
``Publisher`` / ``Subscriber`` pair, a ``ServiceServer`` (``/add_two_ints``) and
``ActionServer`` (``/fibonacci``) a ROS 2 client can drive, and a
``ServiceClient`` / ``ActionClient``.

.. toctree::

   rtps_example

API Reference
-------------

.. include-build-file:: inc/rtps_participant.inc
.. include-build-file:: inc/rtps_pubsub.inc
.. include-build-file:: inc/rtps_service.inc
.. include-build-file:: inc/rtps_action.inc
.. include-build-file:: inc/rtps_message.inc
