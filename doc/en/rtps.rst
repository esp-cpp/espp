RTPS APIs
*********

The ``rtps`` component is a cross-platform foundation for building RTPS
(``Real-Time Publish-Subscribe``) functionality on top of the ESPP ``socket``
component.

This version now implements the first RTPS discovery layer on top of the ESPP
``socket`` component:

- RTPS header and DATA submessage framing helpers
- standard RTPS UDPv4 port calculations
- GUID, entity ID, locator, and sequence number utility types
- SPDP participant announcements using PL_CDR parameter lists
- SEDP publication and subscription announcements for local endpoints
- parsing and tracking of discovered remote participants, writers, and readers
- integration with the shared ``cdr`` component for CDR/PL_CDR payload handling
- a participant transport layer that uses ``UdpSocket`` for metatraffic and
  user-traffic channels
- optional best-effort user-data multicast transport, including endpoint-specific groups

The long-term target is interoperability with ROS 2 nodes over DDS/RTPS,
including best-effort and reliable data flows. Discovery messages are now
standards-shaped, but full wire-compatible ROS 2 interop still needs the
remaining endpoint metadata, matching rules, and reliable RTPS state machines.

Expected Compatibility
----------------------

The table below is deliberately conservative: **expected** means this is the
intended compatibility envelope of the current code, not a claim that every
peer implementation has already been validated in practice.

.. list-table::
   :header-rows: 1

   * - Peer implementation
     - Expected compatibility
     - Notes
   * - ESPP ``rtps`` component / ``python/rtps_host.py``
     - **Yes** for the current scaffold
     - Intended smoke-test path for SPDP, SEDP, and the temporary ``UInt32``
       ``ESPPDATA`` user-data payload.
   * - Generic DDSI-RTPS 2.3 implementations
     - **Partial**
     - SPDP and SEDP messages are standards-shaped, but only the discovery
       slice is implemented today.
   * - ROS 2 nodes backed by Fast DDS
     - **Partial / discovery-targeted**
     - The current discovery messages include ROS 2-relevant participant user
       data such as ``enclave=...;``, but standards-based ROS 2 topic data
       exchange is not finished yet.
   * - ROS 2 nodes backed by Cyclone DDS or other DDS vendors
     - **Partial / unverified**
     - Expected to be limited to the minimal discovery subset if the peer
       accepts the currently emitted parameter set; not validated yet.
   * - Reliable DDS/RTPS endpoints
     - **No**
     - ``HEARTBEAT``, ``ACKNACK``, retransmission windows, and other reliable
       state-machine pieces are not implemented.

How RTPS Works
--------------

RTPS separates *metatraffic* from *user traffic*.

- **Metatraffic** carries discovery and endpoint metadata. In this component,
  that means SPDP participant announcements plus SEDP publication and
  subscription announcements.
- **User traffic** carries application samples. The current ESPP scaffold has a
  temporary best-effort ``UInt32`` user-data path while the standards-based
  ROS 2 data plane is still being completed.

The current ``RtpsParticipant`` implementation opens three UDP sockets when
``start()`` is called:

1. metatraffic multicast receive on the well-known SPDP multicast port
2. metatraffic unicast receive on the participant-specific discovery port
3. user unicast receive on the participant-specific user-data port

It then starts a periodic announce task which multicasts SPDP and unicasts SEDP
endpoint announcements to each discovered peer.

.. mermaid::

   flowchart LR
     App["Application code"] --> Participant["RtpsParticipant"]
     Participant --> SPDP["SPDP participant DATA"]
     Participant --> SEDP["SEDP publication/subscription DATA"]
     Participant --> User["User DATA submessages"]
     SPDP --> MetaMC["Metatraffic multicast"]
     SEDP --> MetaUC["Metatraffic unicast"]
     User --> UserUC["User unicast"]
     MetaMC --> Peer["Remote participant"]
     MetaUC --> Peer
     UserUC --> Peer

Discovery Flow
--------------

At a high level, discovery proceeds like this:

.. mermaid::

   sequenceDiagram
     participant A as Local participant
     participant MC as 239.255.0.1
     participant B as Remote participant
     A->>MC: SPDP DATA(participant GUID, locators, enclave, builtin endpoints)
     MC-->>B: multicast delivery
     B->>MC: SPDP DATA(its participant metadata)
     MC-->>A: multicast delivery
     A->>B: SEDP publication DATA(topic/type/reliability)
     A->>B: SEDP subscription DATA(topic/type/reliability)
     B->>A: SEDP publication/subscription DATA
     Note over A,B: Matching user-data traffic can then use the user-unicast ports

When ``handle_metatraffic_message()`` receives SPDP, it parses:

- the remote participant GUID
- participant name
- the ``enclave=...;`` entry carried in ``PID_USER_DATA``
- built-in endpoint bitmasks
- metatraffic and user-data locators

For SEDP it parses the endpoint GUID, topic name, type name, reliability,
inline-QoS expectation, and unicast locator, then updates the discovered reader
or writer cache.

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

Current ESPP Scope
------------------

The current implementation is intentionally focused on the first
interoperability milestone:

- RTPS message framing and parsing
- SPDP participant discovery
- SEDP publication and subscription discovery
- participant / endpoint caches and discovery callbacks
- simple CDR little-endian serialization helpers for ``std_msgs/msg/UInt32``

The following pieces are **not finished yet**:

- reliable RTPS state machines such as ``HEARTBEAT`` and ``ACKNACK``
- standards-based ROS 2 user-data writers/readers
- full QoS matching beyond the currently emitted discovery parameters

Feature Status
--------------

.. list-table::
   :header-rows: 1

   * - Feature
     - Status
     - Notes
   * - RTPS header / DATA submessage serialize + parse
     - **Implemented**
     - Core message framing is present.
   * - Standard UDPv4 RTPS port mapping
     - **Implemented**
     - Uses the DDSI-RTPS well-known port formula.
   * - SPDP participant announce send/receive
     - **Implemented**
     - Multicast announce plus participant cache updates.
   * - SEDP publication / subscription announce send/receive
     - **Implemented**
     - Local endpoints are announced and remote endpoints are cached.
   * - Participant / endpoint discovery callbacks
     - **Implemented**
     - Exposed through ``on_participant_discovered`` and
       ``on_endpoint_discovered``.
   * - Temporary ``UInt32`` user-data path
     - **Implemented**
     - Uses the current ESPP-specific ``ESPPDATA`` payload, not a
       standards-based DDS sample representation.
   * - Best-effort user-data multicast transport
     - **Implemented**
     - Supports shared participant-level multicast or endpoint-specific
       multicast locators advertised in SEDP; local readers only join the
       multicast groups configured for their topics.
   * - QoS fields emitted in discovery
     - **Partial**
     - Reliability, durability, liveliness, and history parameters are
       advertised in SEDP.
   * - QoS matching / policy enforcement
     - **Not implemented**
     - Remote QoS is parsed, but full writer/reader matching logic is still
       missing.
   * - Standards-based DDS user-data serialization
     - **Not implemented**
     - The current data path is a temporary ESPP scaffold for
       ``std_msgs/msg/UInt32``.
   * - Inline QoS handling
     - **Not implemented**
     - Discovery and user-data handling assume no inline QoS.
   * - Reliable RTPS (``HEARTBEAT``, ``ACKNACK``, resend)
     - **Not implemented**
     - Reliable delivery is not interoperable yet.
   * - Full ROS 2 topic interoperability
     - **Not implemented**
     - Discovery is the current milestone; ROS 2-compatible data
       writers/readers are still pending.

Relevant Specifications
-----------------------

These are the primary standards and references for understanding the current
implementation and the remaining work:

.. list-table::
   :header-rows: 1

   * - Specification
     - Why it matters here
   * - `OMG DDSI-RTPS 2.3 <https://www.omg.org/spec/DDSI-RTPS/2.3/PDF>`_
     - Primary wire-level reference for RTPS headers, DATA submessages, SPDP,
       SEDP, locator encoding, GUIDs, and the UDP port mapping used by this
       component.
   * - `OMG DDS 1.4 <https://www.omg.org/spec/DDS/1.4/PDF>`_
     - Defines the conceptual participant, reader, writer, topic, and QoS model
       that RTPS discovery is advertising.

Example
-------

The :doc:`rtps_example` page demonstrates the current discovery scaffold by:

- computing the RTPS ports for a participant
- building and parsing locally generated SPDP and SEDP messages
- round-tripping a ``UInt32`` value through the CDR helper functions
- registering best-effort and reliable topic endpoints in the participant API

.. toctree::

   rtps_example

API Reference
-------------

.. include-build-file:: inc/rtps.inc
