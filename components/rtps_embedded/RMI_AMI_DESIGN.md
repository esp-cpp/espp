# RMI / AMI design: services (request/reply) and actions (goal server)

Status: **DRAFT for review** (design-first, no code yet).
Scope: add ROS-style **services** (RMI, request/reply) and **actions** (AMI, goal
server) to the `rtps_embedded` engine, in two tracks:

- **Track A — ROS 2 interoperable.** Byte-compatible with FastDDS / `rmw_fastrtps`
  (the stack already validated by the pub/sub interop gate). Other DDS RMWs
  (cyclonedds, connext) are explicitly **later**.
- **Track B — native minimal protocol.** A separate, deliberately lightweight
  request/reply + action protocol for espp↔espp, tuned for embedded entity/memory
  budgets. Does **not** interoperate with ROS and is not meant to.

Guiding invariant (unchanged from the refactor): **never break the existing
FastDDS/ROS 2 pub/sub wire format.** Everything here is *additive* — plain topics
carry no new bytes. New frames get their own golden coverage and their own
interop legs.

---

## 0. The one structural fact that shapes everything

In ROS 2, **services and actions are built entirely on DDS pub/sub**:

- A **service** = two topics (Request + Reply) + a way to correlate each reply to
  its request.
- An **action** = **3 services + 2 topics**, nothing more:
  - services: `send_goal`, `cancel_goal`, `get_result`
  - topics: `feedback`, `status`

So actions add **no new wire primitive**. The entire effort reduces to adding one
capability to the engine — **reliable request/reply with correlation** — and then
writing library code (client/server + an action state machine) on top.

---

## 1. What the engine already provides (verified)

| Capability | Where | Note |
|---|---|---|
| DATA / DATA_FRAG transport | L0 | request/reply payloads ride this unchanged |
| BEST_EFFORT + RELIABLE (HEARTBEAT/ACKNACK) | #660 | services use RELIABLE |
| Reader walks the inline-QoS TLV list | `MessageReceiver.cpp:258` | today it *skips* params; extracting one PID is a bounded add |
| Sender identity per sample | `ReaderCacheChange{writerGuid, sn}` (`Reader.hpp:52`) | the request's **SampleIdentity is already available server-side** — no wire change needed to read it |
| Per-build limit profiles | `RTPS_LIMITS_PROFILE` + Kconfig | services/actions get sizing knobs here |

Gaps to fill:

1. **Reply-side inline-QoS emission** (Track A): the writer has a
   `containsInlineQos` bool but no general parameter emission. We must emit
   `related_sample_identity` on reply frames.
2. **Inline-QoS parse** (Track A): extend the existing TLV skip-loop to capture
   PID `0x8002`.
3. **Correlation/state layer** (both tracks): pending-request table with timeouts;
   action goal state machine.
4. **Name/type mangling helpers** (Track A): `rq/`,`rr/`,`rt/` + `dds_` +
   `_Request_`/`_Response_`. Today mangling is caller-side (tests pass
   pre-mangled names); we add helpers, not a policy change.
5. **Facade surface** to expose sender identity to a service handler (data already
   present internally; just not plumbed through `on_sample`).

---

## 2. Layering

```
L0  RTPS pub/sub (DATA/FRAG, BEST_EFFORT + RELIABLE)          [exists]
        │
L1  Request/Reply primitive + correlation                    [NEW — only wire work]
     ├─ A: sample_identity inline QoS (rmw_fastrtps)
     └─ B: compact in-band correlation header (native)
        │
L2a Service client/server API                                [NEW — no wire work]
        │
L2b Action client/server (state machine over L2a + topics)   [NEW — no wire work]
```

Tracks A and B share L2 *shapes* (same C++ API ergonomics where possible) but have
distinct L1 wire encodings selected at construction / build time.

---

## 3. Track A — ROS 2 interoperable (FastDDS / rmw_fastrtps)

### 3.1 Naming & type mangling

For service `/add_two_ints`, type `example_interfaces/srv/AddTwoInts`:

| Endpoint | DDS topic | DDS type |
|---|---|---|
| request | `rq/add_two_intsRequest` | `example_interfaces::srv::dds_::AddTwoInts_Request_` |
| reply | `rr/add_two_intsReply` | `example_interfaces::srv::dds_::AddTwoInts_Response_` |

Rules: strip leading `/`, prefix `rq`/`rr`, suffix `Request`/`Reply` on the topic;
`::dds_::` infix + `_Request_`/`_Response_` suffix on the type. Namespaced names
keep internal slashes (`/ns/svc` → `rq/ns/svcRequest`). Plain pub/sub topics use
`rt/` + `_` type suffix — the same rule the existing interop already relies on.

### 3.2 Correlation (the crux) — CONFIRMED against a live capture

Verified against a real `rmw_fastrtps` (ROS 2 **Jazzy**, Fast-RTPS vendorId 01.15)
`AddTwoInts` exchange, UDP-only, dissected with tshark. The captured bytes below
supersede the earlier `0x8002` guess.

**The `related_sample_identity` is a 24-byte SampleIdentity carried as inline QoS
under TWO parameter IDs, both present with the identical value:**

```
PID 0x0083  (PID_RELATED_SAMPLE_IDENTITY, OMG DDS-RPC standard)   len 24
PID 0x800f  (eProsima legacy PID_CUSTOM_RELATED_SAMPLE_IDENTITY)  len 24
PID_SENTINEL (0x0001, len 0)

value (24 B, CDR_LE) = GUID (16) + SequenceNumber (8)
    GUID          = guidPrefix (12) + entityId (4)
    SequenceNumber= high (int32) + low (uint32)      // UNKNOWN = high=-1(0xffffffff), low=0
```

Emit BOTH PIDs (for server->ROS-client compat); accept EITHER on receive.

**Both the request AND the reply carry inline QoS** (the earlier "request has no
correlation QoS" note was wrong). Captured pair (client guidPrefix
`010feb7d6c00b8fd0000...`, reply-reader entityId `0x00001304`):

```
REQUEST  rq/add_two_intsRequest  writerSeqNumber=1  payload a=1,b=100
  0x0083/0x800f value = 010feb7d6c00b8fd00000000 00001304 | ffffffff 00000000
                        └─ client reply-reader GUID ─────┘   └ SN = UNKNOWN ┘
REPLY    rr/add_two_intsReply     writerSeqNumber=1  payload sum=101
  0x0083/0x800f value = 010feb7d6c00b8fd00000000 00001304 | 00000000 01000000
                        └─ SAME client GUID ────────────┘   └ SN = 1  ───────┘   (= request's writerSeqNumber)
```

**Correlation algorithm (this is what espp implements):**
- espp as **SERVER**: on a request, capture (a) `related_sample_identity.guid` from
  the request's inline QoS = `client_id_guid`, and (b) the request's RTPS
  `writerSeqNumber` = `req_sn`. Write the reply to `rr/` with inline QoS
  0x0083 + 0x800f = `{ guid = client_id_guid, seq = req_sn }`.
- espp as **CLIENT**: write the request to `rq/` with inline QoS
  `{ guid = <espp's own reply-reader GUID>, seq = UNKNOWN }`, and remember the RTPS
  `writerSeqNumber` used. On each reply, match
  `related_sample_identity.guid == my reply-reader GUID` **and**
  `related_sample_identity.seq == pending writerSeqNumber`.

Reply is **broadcast** on the shared `rr/` topic; every client's reply reader
receives it and filters as above — no directed addressing, pure pub/sub with
client-side filtering, which fits the engine natively. Payload encapsulation is
CDR_LE (0x0001); services are RELIABLE (HEARTBEAT/ACKNACK observed).

Engine impact: the reader already exposes the request's `{writerGuid, sn}` via
`ReaderCacheChange`, but correlation needs the inline-QoS **guid** too, so the
reader must additionally capture the 0x0083/0x800f parameter (extend the existing
inline-QoS TLV loop) and surface it on `ReaderCacheChange`.

> Still to verify before M1 lands (non-blocking for the wire format): whether
> SEDP needs a type hash / `TypeInformation` for `ros2 service list` and matching,
> or plain topic+type-name match suffices. Test against a live node.

### 3.3 Service API (L2a)

```cpp
// Server
participant.add_service_server({
  .service   = "/add_two_ints",
  .type_name = "example_interfaces::srv::dds_::AddTwoInts",   // base; _Request_/_Response_ derived
  .on_request = [](RequestId id, std::span<const uint8_t> req_cdr) -> std::vector<uint8_t> {
      return make_response_cdr(...);                            // returned bytes → reply
  },
});

// Client
auto client = participant.add_service_client({ .service = "/add_two_ints",
                                               .type_name = "...AddTwoInts" });
client.call_async(req_cdr, [](CallResult r, std::span<const uint8_t> resp_cdr){ ... });
auto resp = client.call(req_cdr, 1s);   // optional<vector<uint8_t>>, nullopt on timeout
```

`RequestId` wraps `{Guid_t writerGuid, SequenceNumber_t sn}` (the sample identity).
Reliability defaults to RELIABLE for services.

### 3.4 Actions (L2b) — composition, no wire work

Action `/fibonacci`, type `.../Fibonacci` expands to the standard 5 endpoints:

| Kind | DDS name (mangled) | Payload |
|---|---|---|
| service send_goal | `rq/fibonacci/_action/send_goalRequest` … | `{goal_id: UUID(16), goal}` → `{accepted: bool, stamp}` |
| service cancel_goal | `…/cancel_goal…` | `action_msgs/srv/CancelGoal` |
| service get_result | `…/get_result…` | `{goal_id}` → `{status: int8, result}` |
| topic feedback | `rt/fibonacci/_action/feedback` | `{goal_id, feedback}` |
| topic status | `rt/fibonacci/_action/status` | `action_msgs/msg/GoalStatusArray` |

Goal states (`action_msgs/msg/GoalStatus`): `UNKNOWN=0, ACCEPTED=1, EXECUTING=2,
CANCELING=3, SUCCEEDED=4, CANCELED=5, ABORTED=6`. The action server is a state
machine driving these across the 3 services + status topic; the action client
mirrors it. All of this is library code over L2a + L0.

**Wire format CONFIRMED** against a live `example_interfaces/action/Fibonacci`
capture (ROS 2 Jazzy, order=5, tshark; goal UUID `93beb052…d15de68e`). All CDR_LE,
after the 4-byte encapsulation header. The 3 services correlate exactly like §3.2
(related_sample_identity); actions add NO new wire primitive. Endpoint mangling:
services `rq|rr/<action>/_action/{send_goal,cancel_goal,get_result}{Request,Reply}`,
topics `rt/<action>/_action/{feedback,status}`; types
`<pkg>::action::dds_::<Action>_{SendGoal,GetResult}_{Request,Response}_`,
`_FeedbackMessage_`, and `action_msgs::{srv::dds_::CancelGoal_*, msg::dds_::GoalStatusArray_}`.

| Message | CDR layout (post-encap) | captured bytes |
|---|---|---|
| SendGoal_Request | `goal_id:UUID(16)` + goal | `…UUID… 05000000` (order=5) |
| SendGoal_Response | `accepted:bool(1)`+pad(3) + `stamp{sec:i32,nsec:u32}` | `01000000 a3927e6a dc014325` |
| GetResult_Request | `goal_id:UUID(16)` | `…UUID…` |
| GetResult_Response | `status:i8(1)`+pad(3) + result | `04000000 06000000 00.. 01.. 01.. 02.. 03.. 05..` (SUCCEEDED, [0,1,1,2,3,5]) |
| FeedbackMessage | `goal_id:UUID(16)` + feedback | `…UUID… 03000000 00.. 01.. 01..` (seq len 3) |
| GoalStatusArray | `status_list[]{goal_id:UUID(16), stamp{sec,nsec}, status:i8+pad}` | `01000000 …UUID… a3927e6a 298f4425 01000000` (1 entry, ACCEPTED→…) |

UUID = 16 raw bytes (`unique_identifier_msgs/UUID`), identical across all messages
for one goal - the correlation key for feedback/status/get_result. Arrays are
length-prefixed (uint32) then elements; a leading array count of 1 in
GoalStatusArray is the sequence length.

```cpp
participant.add_action_server({
  .action = "/fibonacci", .type_name = ".../Fibonacci",
  .on_goal   = [](GoalId, std::span<const uint8_t> goal) -> GoalResponse { return ACCEPT; },
  .on_cancel = [](GoalId) -> CancelResponse { return ACCEPT; },
  .execute   = [](GoalHandle h) {
      h.publish_feedback(fb_cdr);
      h.succeed(result_cdr);      // → get_result reply + terminal status
  },
});
```

### 3.5 Engine wire additions (Track A total)

1. `MessageFactory`: emit an inline-QoS ParameterList (`0x8002` + sentinel) on a
   nominated DATA/reply. Additive; gated so plain topics are byte-identical.
2. `MessageReceiver`: in the existing TLV loop, capture `0x8002` into the
   `ReaderCacheChange` (extend struct with an optional `relatedSampleIdentity`).
3. Facade: expose sender `RequestId` to service handlers.

That is the **entire** new wire surface. Everything else is L2 library code.

---

## 4. Track B — native minimal protocol (espp↔espp)

Deliberately a **separate** protocol (per decision): no ROS mangling, no inline-QoS
machinery, no UUIDs, minimal entity count. It still rides the **same L0 RTPS
transport** (DATA + RELIABLE) — "separate" means the correlation/semantics layer,
not a new transport.

### 4.1 Principles

- Correlation in a **compact in-band header** prepended to the CDR payload — no
  inline-QoS emit path needed (just prepend bytes), the simplest possible impl.
- `uint32` ids instead of 16-byte UUIDs / 24-byte sample identities.
- Collapse action's 5 endpoints toward ~3.
- Everything sizeable via `RTPS_LIMITS_PROFILE` (static alloc on esp32).

### 4.2 Native request/reply wire (in-band header)

Reply and request DATA payloads begin with a fixed 20-byte header, then the CDR
body:

```
offset 0   target_prefix : 12 bytes   // RTPS GuidPrefix of the intended peer
offset 12  request_id    : uint32     // client-monotonic
offset 16  op            : uint8      // REQUEST=0, REPLY=1, ERROR=2, CANCEL=3, ...
offset 17  flags         : uint8
offset 18  reserved      : uint16
offset 20  <CDR body>
```

- Request: `target_prefix` = server prefix (from discovery), `op=REQUEST`.
- Reply: `target_prefix` = the requesting client's prefix (learned for free from
  the request's RTPS source), `op=REPLY`.
- A peer accepts a frame iff `target_prefix == myGuidPrefix`; the client then
  matches `request_id` against its pending table. Shared reply topic, client-side
  filter — same pattern as Track A but with a 16-byte in-band key instead of a
  24-byte inline-QoS sample identity, and no PID machinery.

Two topics per service: `es_rq/<svc>` and `es_rr/<svc>` (prefix distinguishes them
from ROS `rq/`/`rr/`, so the two protocols never alias on a shared bus).

### 4.3 Native action wire — collapse the endpoint explosion

| ROS (Track A) | Native (Track B) |
|---|---|
| send_goal service (2 topics) | goal service `es_rq/<a>` + `es_rr/<a>` (2) |
| cancel_goal service (2 topics) | cancel folded into the goal service via `op=CANCEL` (0) |
| get_result service (2 topics) | result delivered as a terminal feedback msg (0) |
| feedback topic (1) | feedback topic `es_fb/<a>` (1) |
| status topic (1) | status folded into feedback `status` field (0) |
| **≈10 endpoints / pair** | **≈3 endpoints one-way, ~4–6 / pair** |

Native feedback message:

```
{ goal_handle: uint32, status: uint8, seq: uint32, payload: <CDR feedback | result> }
```

`status` reuses the ROS state enum values for conceptual parity. A terminal status
(`SUCCEEDED/ABORTED/CANCELED`) carries the result in `payload`; no separate
get_result round-trip.

```cpp
auto h = participant.add_native_action_client({ .action = "grip" });
h.send_goal(goal_cdr,
            on_feedback = [](uint8_t status, std::span<const uint8_t> fb){...},
            on_result   = [](uint8_t status, std::span<const uint8_t> res){...});
h.cancel();
```

### 4.4 Budget vs ROS (the whole point)

- A ROS action client+server pair ≈ **~10 DDS endpoints**, each with its own
  history cache + proxy set under static allocation.
- Native pair ≈ **~4–6**, no UUID/GoalStatusArray types, 20-byte header vs
  inline-QoS + wrapper messages. Concrete esp32 RAM/entity savings, selectable per
  build.

---

## 5. How the two tracks coexist in the codebase

- One request/reply **core** (pending table, timeout, ret/ack) parameterized by an
  **encoding strategy**: `RosSampleIdentity` (A) vs `NativeInbandHeader` (B).
- L2 service/action classes templated/injected on the strategy so the client/server
  logic and the action state machine are written **once**.
- Public API: `add_service_server` / `add_service_client` /
  `add_action_server` / `add_action_client` with a `Wire::Ros | Wire::Native`
  selector (default `Ros` on host, `Native` where interop isn't compiled). esp32
  can compile out Track A entirely (Kconfig), like fragmentation.
- Topic-prefix disjointness (`rq/`/`rr/` vs `es_rq/`/`es_rr/`) means both can run on
  one bus without aliasing.

---

## 6. esp32 considerations

- New Kconfig: `RTPS_ENABLE_SERVICES`, `RTPS_ENABLE_ACTIONS`,
  `RTPS_ENABLE_ROS_RPC` (Track A), each opt-out-able; native-only build drops all
  ROS mangling/inline-QoS code.
- Limit knobs in the profiles: max concurrent services, max in-flight requests per
  client, pending-table depth, per-goal state slots. Embedded profile static;
  host/host_large dynamic (reuse the `StorageArray` policy).
- Timeouts + pending-table eviction must be bounded/static on embedded (no
  unbounded growth from lost replies).

---

## 7. Testing & gates (extends the existing discipline)

1. **Golden** — new byte-for-byte captures for: a Track-A reply frame with the
   `0x8002` inline QoS; a Track-B request/reply header. Never regenerate existing
   golden.
2. **Docker interop matrix** — new legs:
   - espp service **server** ↔ `ros2 service call` client
   - espp service **client** ↔ rclpy service server
   - espp action **server** ↔ `ros2 action send_goal` (feedback + result + cancel)
   - espp action **client** ↔ rclpy action server
3. **Host loopback** — Track-B service + action request/reply/cancel/feedback,
   in-process, byte-exact + no-skip under concurrency (mirrors `rtps_facade_backlog`).
4. **esp32 build** — services on/off, actions on/off, ROS-RPC on/off, frag on/off.

---

## 8. Milestones (stacked PRs)

1. **M1 — Track A request/reply core. ✅ DONE** (branch `feat/rtps-services`).
   - M1.1 `rpc/service_naming.hpp` mangling + host test (7/7).
   - M1.2 `rpc/sample_identity.hpp` + `addSubMessageDataWithRelatedSampleIdentity`
     emit + `MessageReceiver` parse into `ReaderCacheChange`; golden section
     `data_related_sample_identity` (all prior golden bytes unchanged).
   - M1.3 send-path plumbing: `CacheChange` carries the identity; both writers
     branch to the RSI emit; plain pub/sub byte-identical.
   - M1.4 facade `add_service_server` / `add_service_client` + `ServiceClient`
     (sync `call` + `call_async`), pending-request correlation; in-process
     `rtps_service_loopback`.
   - M1.5 live ROS 2 interop **both directions** (`ros2 service call` -> espp,
     and espp client -> rclpy server). Final gate: interop **16/16**.
   The actual correlation is inline QoS PIDs **0x0083 + 0x800f** (not 0x8002);
   both request and reply carry it. See §3.2.
2. **M2 — Track A actions.** State machine over M1 + feedback/status topics. Gate:
   both action interop directions incl. cancel.
3. **M3 — Track B native RMI/AMI.** In-band header, lean service + action, host
   loopback gates, esp32 budget profiles.
4. **M4 — Consolidation.** Shared L2 over both strategies; Kconfig compile-out;
   docs + examples (C++ and the Python facade).

Track B (M3) can precede M1/M2 if the immediate need is espp↔espp — it has no
external dependency and no interop gate to satisfy.

---

## 9. Open verification items

- [x] Live `rmw_fastrtps` service capture → **done** (§3.2). Correlation is inline
      QoS PIDs **0x0083 + 0x800f** (24-byte SampleIdentity, CDR_LE), on BOTH
      request and reply. Corrected the earlier 0x8002 / no-request-QoS guesses.
- [x] Target ROS distro → **Jazzy** (Fast-RTPS vendorId 01.15).
- [x] Does SEDP need service-specific discovery attributes (type hash,
      `TypeInformation`) for `ros2 service list` / matching? **Answered: NO** (M1.5).
      Plain rq/rr topic + `_Request_`/`_Response_` type-name matching suffices;
      the espp service even appears in `ros2 service list` and a live
      `ros2 service call` succeeds against it, with no type-hash exchange.
- [ ] Confirm `action_msgs` / `unique_identifier_msgs` CDR layouts (UUID = 16 raw
      bytes; GoalStatus/GoalStatusArray) for Track A actions (M2).
- [ ] Decide default `Wire` per platform and whether Track A is default-off on esp32.
