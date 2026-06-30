# Design: Reliable RTPS (HEARTBEAT / ACKNACK) for user data

Status: **complete (pending final interop sign-off).** Phases 0–4 are
implemented and hardware-validated against Fast DDS/RTPS:

- Phase 0 — submessage codecs (`SequenceNumberSet`, `HEARTBEAT`, `ACKNACK`,
  `INFO_DST`, `GAP`) + dispatch seam.
- Phase 1 — reliable writer: history cache + HEARTBEAT emission.
- Phase 2 — reliable reader: dedup + in-order delivery + ACKNACK generation.
- Phase 3 — writer-side retransmission on ACKNACK, for **both** user-data
  writers and the builtin **SEDP** writers, plus `GAP` for evicted/irrelevant
  samples (emitted by the writer, honored by the reader).
- Phase 4 — hardening: builtin SEDP reliability (stable SEDP sequence numbers +
  SEDP HEARTBEATs so a reliable peer recovers a missed announcement), per-reader
  `DATA` `readerId` addressing, multi-homed multicast interface selection, and a
  GUID-keyed `DiscoveryDb` for discovery state.

Best-effort behavior is unchanged for endpoints that advertise `BEST_EFFORT`.
Reliable recovery now works in both directions: an espp reliable reader recovers
lost samples from a DDS/ROS 2 writer, and an espp reliable writer answers a
peer's NACKs (and GAPs samples it can no longer provide).

## Goal

Add interoperable `RELIABLE` delivery for user data: a stateful writer that
retains a history of sent samples and retransmits on request, and a stateful
reader that detects gaps and requests retransmission. The reliable handshake
runs only between endpoints that *both* advertise `RELIABLE` (discovered via
SEDP). DDS/ROS 2 interop is a first-class requirement.

## Background — what exists today

The user-data path is **stateless** (see `components/rtps/src/rtps.cpp`):

- `publish()` (rtps.cpp:1150) builds one `DATA` submessage with a per-writer
  monotonic sequence number (`next_user_data_sequence_number`) and sends it once
  to each destination. There is no history cache and no retransmit.
- `handle_user_message()` (rtps.cpp:1427) parses `DATA`, resolves the topic via
  the remote writer GUID, and invokes matching reader callbacks. For a reliable
  writer it logs *"ACKNACK/HEARTBEAT is not implemented yet"* (1473) and delivers
  anyway. There is no dedup, ordering, or per-writer receive state.
- `Message::parse` (rtps.cpp:793) decodes the generic submessage frame
  (kind/flags/length/payload); only `DATA` is interpreted. `HEARTBEAT`/`ACKNACK`
  exist in `SubmessageKind` (rtps.hpp:124) but are never parsed or generated.
  `INFO_DST` is never emitted.

What we can build on:

- Per-writer monotonic sequence numbers and writer-GUID-based receive routing.
- `EndpointProxy` (rtps.hpp:202) already carries the remote `guid`,
  `reliability`, `unicast_locator`, and `multicast_locators`, and
  `ParticipantProxy` (rtps.hpp:191) carries the remote `address` + `ports`, so
  the addressing data needed for stateful matching already exists in discovery.
- `parse_data_submessage` honors submessage endianness (the `E` flag) and skips
  inline QoS — the new parsers mirror that.

## Wire formats

All submessages we emit are little-endian (`E` flag set), consistent with the
existing `DATA` path. Parsers honor the `E` flag for the submessage body.

### SequenceNumber (8 bytes)
`high : int32` then `low : uint32` (each in submessage endianness). Already
handled by `ByteWriter::append_sequence_number_le` / `ByteReader::read_sequence_number`.

### SequenceNumberSet
`bitmapBase : SequenceNumber (8)`, `numBits : uint32 (4)`,
`bitmap : uint32[ceil(numBits/32)]`. Bit *i* (MSB-first within each word) set ⇒
`bitmapBase + i` is requested/missing. `numBits` is 0..256. This is the fiddliest
piece — unit-tested in isolation (empty set, single bit, 256-bit max, base
alignment, word boundaries).

### HEARTBEAT (0x07)
Body: `readerId(4) writerId(4) firstSN(8) lastSN(8) count : uint32(4)`.
Flags: `E` (endian, bit 0), `F` (final — no response required, bit 1),
`L` (liveliness, bit 2). `firstSN..lastSN` is the inclusive range of sequence
numbers currently available in the writer's history. `count` is monotonic per
writer (stale-heartbeat detection).

### ACKNACK (0x06)
Body: `readerId(4) writerId(4) readerSNState : SequenceNumberSet count : uint32(4)`.
Flags: `E`, `F` (final). `readerSNState.bitmapBase` is the lowest sequence number
the reader still needs; the bitmap marks which of `[base, base+numBits)` are
missing. An empty set with `base = lastSN + 1` is a positive ack of everything.
`count` is monotonic per reader.

### INFO_DST (0x0e)
Body: `guidPrefix(12)`. Prefixes a directed HEARTBEAT/ACKNACK so the receiving
participant routes it to the right entity. Required for robust DDS interop.

## Architecture

### Writer side (`WriterReliableState`, per local reliable writer)
- `history : std::map<int64_t, std::vector<uint8_t>>` — CDR payloads keyed by SN,
  capped to a KEEP_LAST depth (drop oldest, advance `first_sn`).
- `first_sn`, `last_sn`, `heartbeat_count`.
- Per matched reliable reader: highest acked SN (`ReaderProxy`), to know when a
  sample can be purged and whether a heartbeat still needs a response.
- On `publish()`: store sample → send `DATA` → send a (non-final) `HEARTBEAT`
  (INFO_DST + HEARTBEAT) to each matched reliable reader's unicast locator.
- Periodic `HEARTBEAT` (extend `announce_task_` or a dedicated heartbeat task,
  with jitter) while any reader has unacked samples.
- On `ACKNACK`: resend the requested SNs still in history to the requesting
  reader's locator; advance that reader's acked watermark (`base - 1`).

### Reader side (`ReaderReliableState`, per (local reader, remote writer GUID))
- Highest-contiguous received SN + a bounded out-of-order/reorder set,
  `last_heartbeat_count`, `acknack_count`.
- On `DATA` from a reliable writer: record the SN, **dedup**, deliver in SN order
  (bounded reorder buffer).
- On `HEARTBEAT`: compute missing SNs in `[firstSN, lastSN]`, reply with `ACKNACK`
  (INFO_DST + ACKNACK) addressed to the writer's user-unicast endpoint (resolved
  from `discovered_writers_ → participant_guid → ParticipantProxy.address/ports.user_unicast`,
  **not** the raw `Socket::Info` source port). Respond to non-final heartbeats
  even when caught up so the writer can stop repeating / purge.

### Addressing
- Writer → reader (HEARTBEAT, DATA resend): `EndpointProxy.unicast_locator` of the
  matched remote reader.
- Reader → writer (ACKNACK): the writer's participant user-unicast address+port.
- Both are sent on the existing `user_unicast_receiver_` socket. Phase 1–3 are
  unicast-only; reliable-over-multicast is a later extension.

### Concurrency
The history cache and the reader/writer proxy maps are touched by the publish
path (app thread), the receive path (one or more receive tasks), and the
heartbeat/retransmit timer. Guard them with a dedicated `reliable_mutex_`
(distinct from `mutex_` / `receivers_mutex_` / `sequence_mutex_`); never hold it
nested under `mutex_`. Document the lock ordering as: `mutex_` (discovery state)
is taken to snapshot endpoints, released, then `reliable_mutex_` for reliable
state — matching the existing snapshot-then-act pattern in
`build_user_send_configs`.

### Config additions
- `WriterConfig.history_depth` (KEEP_LAST, default e.g. 16); replaces the
  `kHistoryKeepLast = 0` placeholder (rtps.cpp:54) and feeds the SEDP history QoS.
- `Config.heartbeat_period` (default ~200 ms) and `Config.reliable_reorder_depth`.

## Phase plan

- **Phase 0 — submessage codecs + dispatch seam** *(this PR)*. Add the
  `SequenceNumberSet` codec, `HEARTBEAT`/`ACKNACK`/`INFO_DST` build + parse
  helpers (internal, endian-aware), and dispatch `HEARTBEAT`/`ACKNACK` in
  `handle_user_message` to `handle_heartbeat_submessage` / `handle_acknack_submessage`
  handlers (initially logging at debug). No behavior change for best-effort; the
  reliable downgrade warning stays until Phase 2. Round-trip tests for the codecs.
- **Phase 1 — reliable writer** *(done)*: `WriterReliableState` history cache
  (keyed by SN, bounded by `WriterConfig.history_depth`), sample cached on
  `publish()`, and HEARTBEAT (INFO_DST + HEARTBEAT) emitted after publish and
  periodically (`Config.heartbeat_period`, `heartbeat_task_`) to each matched
  reliable reader's unicast locator. SEDP now advertises the real history depth.
  Guarded by `reliable_mutex_`. The ACKNACK-driven retransmission response is
  Phase 3.
- **Phase 2 — reliable reader** *(done)*: `ReaderReliableState` keyed by
  "<reader index>#<writer GUID>"; `deliver_reliable_sample()` dedups and delivers
  in order via a bounded reorder buffer (`Config.reliable_reorder_depth`);
  `send_acknack_for_heartbeat()` replies to a HEARTBEAT with an `INFO_DST +
  ACKNACK` carrying the `SequenceNumberSet` of missing SNs (or a positive ack),
  addressed to the writer's unicast locator (fallback: participant user-unicast).
  Handles writer-purged gaps (advance past lost SNs) and stale heartbeats.
  Reliable handshake runs only when both endpoints advertise RELIABLE; the
  downgrade warning is removed.
- **Phase 3 — writer retransmission** *(done)*: on ACKNACK, a writer resends the
  NACKed SNs still in history to the requesting reader. Shared
  `build_directed_data_message` (INFO_DST + DATA addressed to the requesting
  reader) drives both `retransmit_user_data` (from the per-writer history) and
  `retransmit_sedp` (rebuilt deterministically from `writers_`/`readers_` via the
  stable index-based SEDP sequence numbers — no separate SEDP cache). Samples no
  longer in history are answered with a `GAP`; the reader's `apply_gap` advances
  its frontier over the irrelevant SNs and releases anything buffered behind them.
- **Phase 4 — hardening** *(done)*: builtin SEDP reliability (stable SEDP SNs +
  SEDP HEARTBEATs so a reliable peer ACKNACKs and we retransmit a missed
  announcement), per-reader `DATA` `readerId` addressing, `stop()` cleanup of all
  reliable state, and bounded reorder/irrelevant buffers. (Remaining nice-to-haves:
  initial heartbeat on reader match, stale-count rejection, jittered timers.)
- **Phase 5 — interop validation** *(in progress)*: hardware-validated publishing
  from an ESP32-P4 (Ethernet) to a Fast RTPS subscriber, including the SEDP
  HEARTBEAT/ACKNACK exchange. Still TODO: induced packet-loss tests and Cyclone.

## Design decisions
- **History model**: KEEP_LAST depth per writer (bounded memory on embedded), not
  KEEP_ALL, by default.
- **Delivery**: dedup + monotonic in-order with a small bounded reorder buffer.
- **INFO_DST**: emitted from the start (needed for DDS interop).
- **Endianness**: emit little-endian; parse honoring the `E` flag.
- **Scope**: unicast reliable first.

## Out of scope (later)
- Durability beyond VOLATILE (TRANSIENT_LOCAL replay to late joiners).
- Fragmentation (`DATA_FRAG`) for samples larger than the MTU.
- Full QoS-incompatibility reporting.
- Reliable-over-multicast (the reliable handshake is unicast-only).
