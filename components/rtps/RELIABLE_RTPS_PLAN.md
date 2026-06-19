# Plan: Reliable RTPS (HEARTBEAT / ACKNACK) — separate PR

Status: **planned, not started.** This is a fast-follow to the best-effort
CDR-over-RTPS data path and should land as its own PR after that work is merged
and tested.

## Goal

Add interoperable RELIABLE delivery for user data: a stateful writer that
retains history and retransmits, and a stateful reader that detects gaps and
requests retransmission. Best-effort behavior must remain unchanged for
endpoints that advertise `BEST_EFFORT`.

## Why this is a bigger change

The current user-data path is **stateless** — `publish()` builds a `DATA`
message and fires it once; the reader dispatches whatever arrives. Reliability
requires **per-endpoint state machines** and a **writer history cache**, plus
new submessages and timers. The reliable handshake only runs between endpoints
that both advertise `RELIABLE` (discovered via SEDP).

## What Tier 1 already gives us

- Per-writer monotonic sequence numbers (`next_user_data_sequence_number`).
- Writer-GUID-based receive routing and discovered-endpoint reliability flags.
- Standard `DATA` submessage build/parse and the CDR payload path.
- `parse_data_submessage` already honors endianness and skips inline QoS.

## Work breakdown

### 1. Submessage codecs (~1 day)
- `HEARTBEAT` (0x07): `{readerId, writerId, firstSN, lastSN, count}` + flags
  (Final `F`, Liveliness `L`).
- `ACKNACK` (0x06): `{readerId, writerId, readerSNState, count}` + Final flag.
- `GAP` (0x08): `{readerId, writerId, gapStart, gapList}` (can defer to a
  follow-up; needed for irrelevant/removed samples).
- **`SequenceNumberSet`** encoding/parsing: `{bitmapBase (SN), numBits (u32),
  bitmap (ceil(numBits/32) u32 words)}`. This is the fiddliest piece — unit-test
  it in isolation (round-trip + boundary cases: empty set, 256-bit max, base
  alignment).
- Wire each new submessage through `Message::serialize` / `Message::parse`
  (they already carry arbitrary submessages; only the payload codecs are new).

### 2. Reliable writer (~2–3 days)
- **History cache**: retain `(SN -> serialized DATA)` until acked by all matched
  reliable readers; cap by a configurable depth (KEEP_LAST) and drop oldest.
- **Periodic HEARTBEAT** task (reuse the `Task` pattern): announce
  `(firstSN, lastSN)` with an incrementing count to each matched reliable
  reader's unicast locator.
- **ReaderProxy** per matched reliable reader: highest contiguous acked SN +
  requested (nacked) set.
- **On ACKNACK**: advance the acked watermark, resend the nacked SNs still in
  history, dedup by `count`.
- Send a final HEARTBEAT (F flag) after a burst to prompt a prompt ACKNACK.

### 3. Reliable reader (~2–3 days)
- **WriterProxy** per matched reliable writer: highest contiguous received SN +
  missing set + last heartbeat count.
- **On HEARTBEAT**: compute missing SNs in `(firstSN, lastSN)`, reply with an
  `ACKNACK` carrying the `SequenceNumberSet` of missing SNs (or an ack-all when
  caught up); honor the F/L flags and dedup by count.
- **In-order delivery**: buffer out-of-order samples and release to `on_sample`
  in SN order; bound the reorder buffer.
- Send ACKNACK to the writer's unicast locator (from discovery).

### 4. Glue, timers, safety (~1–2 days)
- Run the handshake only when both endpoints advertise `RELIABLE`.
- Heartbeat period / nack-response / retransmit timers via `Task`; jitter to
  avoid sync storms.
- Thread-safety: the history cache and proxy maps are touched by the publish
  path, the receive path, and timer tasks — guard with a dedicated mutex
  (distinct from `mutex_`/`receivers_mutex_`; document lock ordering).
- Remove the "reliable not implemented; sending best-effort" downgrade warnings
  in `publish()` and `handle_user_message`.

### 5. Interop validation (~2–3 days, often dominates)
- Round-trip against another espp participant first.
- Then Fast DDS / ROS 2 with Wireshark: confirm HEARTBEAT/ACKNACK exchange,
  `SequenceNumberSet` bitmaps, retransmission, and that dropped packets recover.
- Test packet loss explicitly (drop a percentage in a test transport or via
  `tc netem`).

## Estimate

- **Minimal happy-path** (periodic heartbeat, retransmit-on-any-nack,
  ack-up-to-lastSN, no GAP): ~3–5 days.
- **Robust** (proper bitmap nacks, GAP, counts, reorder buffer, edge cases):
  ~1.5–2.5 weeks including interop debugging.

## Suggested PR sequencing

1. Submessage codecs + `SequenceNumberSet` with unit tests (item 1) — small,
   reviewable, no behavior change.
2. Reliable writer (item 2).
3. Reliable reader (item 3) + glue (item 4) + interop validation (item 5).

## Out of scope (later)

- Durability beyond VOLATILE (TRANSIENT_LOCAL history replay to late-joiners).
- Full QoS matching/incompatibility reporting.
- Fragmentation (`DATA_FRAG`) for samples larger than the MTU.
