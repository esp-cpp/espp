# RTPS interop harness (FastDDS, CycloneDDS / ROS 2)

The Phase 0 safety net from `../REFACTOR_PLAN.md`: every refactor phase must keep
this matrix green.

## Run

```bash
cd components/rtps/interop
./run.sh
```

Requires docker. One container (`ros:jazzy-ros-base` = FastDDS + rmw_fastrtps_cpp +
`ros2` CLI, plus rmw_cyclonedds_cpp) runs everything in a single network namespace, so RTPS multicast works
unconditionally. `ROS_DISTRO=humble ./run.sh` runs the same matrix on
`ros:humble-ros-base` (the Dockerfile adds gcc 13 and a newer CMake there);
CI runs both. The repo is bind-mounted and copied to a container-local tree
before building (espp is installed into a container-local staging prefix and the
pc tests `find_package` it from there), so your host build artifacts are never
touched.

## Matrix

| test | what it proves |
|---|---|
| build | the engine + espp lib build on linux |
| golden | wire-format bytes unchanged (see `pc/tests/rtps_golden.cpp`) |
| locator_subnet | of the unicast locators a peer on several networks announces (SEDP and SPDP), the one on our subnet is kept (see `pc/tests/rtps_locator_subnet.cpp`) |
| sedp_inline_qos_dispose | a peer's dispose of an endpoint (SEDP) or of its participant (SPDP) - a DATA whose inline QoS carries `PID_STATUS_INFO` + `PID_KEY_HASH`, as Fast DDS, Cyclone DDS and espp write it, in each of the three forms - unmatches the endpoint / forgets the participant at once instead of at lease expiry (see `pc/tests/rtps_sedp_inline_qos_dispose.cpp`) |
| sedp_history_ring, sedp_history_overflow | the SEDP writers never overwrite a live endpoint announcement when their history is full: a sent dispose is given up first, else the announcement is refused and the endpoint creation fails loudly (the ring policy on a static ring, `pc/tests/rtps_sedp_history_ring.cpp`, a unit that links no engine object); a refused creation gives its pooled endpoint back, and an endpoint whose announcement is gone can still be deleted with its slot reused (`pc/tests/rtps_sedp_history_overflow.cpp`) |
| facade_matched_counts | `RtpsParticipant::matched_writers()` / `matched_readers()` over a loopback: one remote writer counts once across two local readers on the topic, and removing the writer (which announces the SEDP dispose) brings the count to zero within a couple of seconds, not at lease expiry (`pc/tests/rtps_facade_matched_counts.cpp`) |
| gap_before_data | a reliable reader applies a GAP only for sequence numbers the writer has announced (HEARTBEAT lastSN or a DATA): Fast DDS sends its SEDP readers a GAP for the next number after an endpoint deletion and then writes that number for the next announcement, which used to leave a publisher-churning peer unmatched after its first deletion (`pc/tests/rtps_gap_before_data.cpp`; the churn case is the live check) |
| acknack_heartbeat_reply | a reliable writer answers an ACKNACK that requests nothing from a reader that has not acknowledged its last change with a HEARTBEAT at once (RTPS 2.5 8.4.15.5): Cyclone DDS's pre-emptive ACKNACK is base 1 / count 0, and waiting for the periodic heartbeat let a reply written for the new reader be announced by the first heartbeat it processed, which its VOLATILE reader took for pre-match history (`pc/tests/rtps_acknack_heartbeat_reply.cpp`; the QEMU `netem_loss_service_cyclone` case is the live check) |
| service_reply_waits_for_reader | a service reply is held until the requester's reply reader is matched to the reply writer (`RtpsParticipant::Config::service_reply_match_timeout`, 1 s by default, then written anyway): a client sends its request as soon as its own side has matched, and a reply written before this side matches its reply reader is never sent to it - the first HEARTBEAT announces it as pre-match history, which a VOLATILE reader skips (the QEMU `netem_loss_service_cyclone` case, one run in six). rmw_fastrtps / rmw_cyclonedds servers wait for the same match. Hand-crafted SPDP/SEDP/request datagrams from a socket that is the peer's locator (`pc/tests/rtps_service_reply_waits_for_reader.cpp`) |
| acknack_coalescing | a reliable reader answers a HEARTBEAT it misses nothing for only when the writer's FinalFlag is clear, and while it is missing samples answers (HEARTBEATs and GAPs) at most once per `SFR_ACKNACK_MIN_INTERVAL_MS` per writer, as Fast DDS and Cyclone DDS coalesce theirs: answering every HEARTBEAT at once let a reader that fell behind a 20 Hz Cyclone writer spiral, each stale HEARTBEAT it worked through bringing a GAP + HEARTBEAT back faster than it drained its backlog (the QEMU soak's Cyclone cycles lost their 30 KB samples and service calls) (`pc/tests/rtps_acknack_coalescing.cpp`) |
| nack_frag | a reliable reader that holds part of a fragmented sample answers the writer's HEARTBEAT with a NACK_FRAG for the fragments it is missing, in the same datagram as its ACKNACK (RTPS 2.5 8.3.7.10): Cyclone DDS answers an ACKNACK naming a fragmented sample with its first fragment only and leaves the rest to NACK_FRAG, so a 30 KB sample a Cyclone publisher wrote just before the device's reader matched it never completed and held the reassembly slot against every sample after it (the QEMU soak's Cyclone cycles) (`pc/tests/rtps_nack_frag.cpp`) |
| receiver_malformed | ACKNACK / GAP / HEARTBEAT / DATA / DATA_FRAG submessages declared shorter than their fixed body, longer than the datagram, truncated, or with a `numBits` past the set are rejected without reading past their bytes, and a valid ACKNACK / DATA afterwards still takes effect (see `pc/tests/rtps_receiver_malformed.cpp`) |
| unknown_reader_fanout | two local readers on one topic are both matched to a remote writer, and a DATA / HEARTBEAT / GAP addressed to `ENTITYID_UNKNOWN` (multicast, or a writer sending to several readers; Cyclone DDS's heartbeats) reaches both; two local writers both get a remote reader (see `pc/tests/rtps_unknown_reader_fanout.cpp`) |
| delete_reader_callback_domain_api | a reader is removed while its on_sample is running and that callback creates a writer: no deadlock between the Domain mutex and the callback drain (see `pc/tests/rtps_delete_reader_callback_domain_api.cpp`) |
| proxy_pool_full_match | a remote endpoint that finds no room in a reader's / writer's proxy pool stays in the registry of known remote endpoints (with a warning naming `NUM_WRITER_PROXIES_PER_READER` / `NUM_READER_PROXIES_PER_WRITER`) and matched when a proxy is freed by a dispose or a departing participant; on the host, whose pools grow, the same run pins that every endpoint is matched at once (see `pc/tests/rtps_proxy_pool_full_match.cpp`) |
| frag_slot_isolation | a DATA_FRAG from a writer the reader never matched cannot evict the single reassembly slot: a stray fragment in the middle of a legitimate sample leaves it delivered byte-exact (see `pc/tests/rtps_frag_slot_isolation.cpp`) |
| dispose_retention | a burst of `HISTORY_SIZE_STATEFUL` dispose-after-write changes all stay in the writer's history until their 4 s retention ends; the retention queue is sized like the history and gives up the oldest, never the newest, when it fills (see `pc/tests/rtps_dispose_retention.cpp`) |
| loopback | espp<->espp discovery + delivery in one process |
| espp_pub->ros2_echo | espp reliable writer -> ROS 2 subscriber (`rt/chatter`, `std_msgs::msg::dds_::String_`) |
| ros2_pub->espp_sub | ROS 2 publisher -> espp reliable reader |
| espp_be_pub->ros2_be_echo | best-effort pairing |
| ros2_pub->espp_sub_liveliness, espp_pub->ros2_sub_liveliness | samples keep arriving 22 s in, i.e. past a remote participant lease (CycloneDDS: 10 s). Only samples after that point count (`rtps_interop_sub ... count_after_s`, `ros2_count_subscriber.py`), so a peer whose SPDP re-announcements were not delivered - and whose lease therefore expired, dropping every matched endpoint - fails. CycloneDDS addresses those re-announcements to readerId ENTITYID_UNKNOWN; the FastDDS pass is the control. Only the espp-publisher leg detects a dropped peer: a HEARTBEAT that reaches a matched espp reader also refreshes the sending participant's liveliness (`MessageReceiver::processHeartbeat`), so a CycloneDDS *writer* keeps its participant alive through its heartbeats even when its SPDP re-announcements are lost, while an espp writer only gets ACKNACKs back, which do not refresh it, and the peer expires |
| many_participants_late_joiner | twelve unrelated ROS 2 participants are on the domain before espp starts, then the publisher espp needs joins last and must still be matched. Guards the remote-participant pool (`SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`, 32 on the host profile built here, 10 on the embedded profile - raise it via menuconfig for a domain this busy): with the pool full, a newly announcing participant evicts the oldest one none of espp's endpoints match (`RTPS_SPDP_EVICT_UNUSED_PARTICIPANTS`, on by default, one eviction per announcement period, logged); with it off a participant beyond the pool is ignored silently |
| ros2_pub_churn->espp_sub_unmatch | a ROS 2 node creates, uses and destroys a publisher five times in 25 s and exits (`ros2_churn_publisher.py`): the espp subscriber must see five matched->unmatched transitions, each after samples arrived, within 40 s (`rtps_interop_churn`, polling `RtpsParticipant::matched_writers()`). Only a peer's dispose can do that; a writer dropped on lease expiry alone (20 s Fast DDS, 10 s Cyclone DDS) would give at most two |
| rpc_stop_race, rpc_pending_eviction, rpc_empty_reply | the service facade: a client handle used while, and after, its participant stops returns 0 / false instead of touching freed endpoints (`pc/tests/rtps_rpc_stop_race.cpp`); a client's pending table is bounded by `ServiceConfig::request_timeout` / `max_pending_requests` and stop() releases every waiter - `call()` nullopt at once, `call_future()` nullopt rather than broken_promise (`rtps_rpc_pending_eviction.cpp`); a handler returning nothing sends nothing, the client times out (`rtps_rpc_empty_reply.cpp`) |
| typed_action_alignment, action_cancel_codes, action_goal_flood | the action facade: a typed Result with int64 / float64 fields is carried as one `GetResult_Response` message, byte-exact against the hand-written layout (`rtps_typed_action_alignment.cpp`); a cancel is answered with the `action_msgs/srv/CancelGoal` verdict - the goals canceling, ERROR_REJECTED / UNKNOWN_GOAL_ID / GOAL_TERMINATED, cancel-all (`rtps_action_cancel_codes.cpp`); `ActionConfig::max_goals_in_flight` caps the execute threads, duplicate ids are rejected, a finished goal is retired after `result_timeout` (`rtps_action_goal_flood.cpp`) |
| action_long_goal | a goal that runs longer than the service client's `request_timeout` still delivers its result: the action's get_result request is never timed out by the client (`ActionConfig::request_timeout` covers send_goal and cancel only) (`pc/tests/rtps_action_long_goal.cpp`) |
| espp_service_server<-ros2_client, espp_service_client->ros2_server | AddTwoInts both ways against a live ROS 2 client (`ros2 service call`) and server (rclpy) |
| espp_action_server<-ros2_client, espp_action_client->ros2_server | Fibonacci both ways: send_goal, feedback, deferred get_result |
| espp_service_client_x8->ros2_server | eight requests in flight at once from one client, each with its own operands: every reply must land on the request it answers (`rtps_service_interop_client ... 8`) |
| two_espp_clients->ros2_server | two espp clients call one ROS 2 server at the same time: a client must take only the replies addressed to it |
| espp_service_client->late_ros2_server | the client is up and calling before any server exists; the ROS 2 server joins 8 s later and the call must still succeed |
| espp_trigger_server<-ros2_client | `std_srvs/srv/Trigger`: the smallest request a ROS 2 client can send (an empty message, one placeholder byte) and a 48000-character reply in a single DATA (`rtps_trigger_interop_server`) |
| espp_action_server<-ros2_cancel, espp_action_server<-ros2_reject | goal paths `ros2 action send_goal` cannot drive (`ros2_fibonacci_client.py`): a goal cancelled at its first feedback ends CANCELED with the partial sequence, and the cancel is answered `ERROR_NONE` with the goal listed in `goals_canceling` (what rclpy's client checks); a goal the server refuses (order 0) is reported rejected |
| espp_action_server<-ros2_cancel_unknown, espp_action_server<-ros2_cancel_finished | the other CancelGoal verdicts (`action_msgs/srv/CancelGoal`): a goal id the server never saw is answered `ERROR_UNKNOWN_GOAL_ID`; a goal run to completion, its result fetched, is still known for `ActionConfig::result_timeout` and answered `ERROR_GOAL_TERMINATED` |
| cyclone: ... | every ROS 2 case above again on `rmw_cyclonedds_cpp`: the pub/sub pairs (reliable and best-effort, both directions), liveliness, the busy domain, the publisher churn, and the six service/action cases (including the two cancel verdicts). rmw_cyclonedds_cpp correlates a request with its reply through a `{guid, seq}` header inside the payload rather than the `related_sample_identity` inline QoS rmw_fastrtps_cpp uses; espp detects the peer's vendor and answers in its form (`pc/tests/rtps_cyclonedds_rpc_header.cpp` pins the codec) |
| mixed_rmw_clients->espp_service_server | a Fast DDS client and a Cyclone client call one espp service at the same time, three rounds: the server answers each request in its client's form, with no cross-talk between their replies |

## In QEMU: the esp32 build of the engine

Everything above runs the desktop build of the engine. `run_qemu.sh` runs the
**esp32 build** - the embedded configuration profile, lwIP, FreeRTOS - in
Espressif's QEMU inside the same container, on a tap device that is the
container's only interface (`docker run --network none --cap-add NET_ADMIN
--device /dev/net/tun`), against the same peers, and asserts on the device's
serial log. The firmware is `qemu/` (see its README for the build and the
`RTPSTEST` markers); `.github/workflows/rtps_qemu.yml` builds it with the IDF
action and runs the matrix on every rtps / qemu / ethernet / socket change.

```bash
cd components/rtps/interop/qemu
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ci.defaults" build
(cd build && python -m esptool --chip esp32 merge-bin --fill-flash-size 4MB -o qemu_flash.bin @flash_args)
cd .. && ./run_qemu.sh
```

| case | what the target proves |
|---|---|
| device_boot, device_ping | the engine comes up on lwIP over the emulated OpenCores MAC |
| device_pub->ros2_echo, ros2_pub->device_sub (+ best-effort pair) | delivery both ways, FastDDS and CycloneDDS |
| ros2_pub->device_sub_liveliness | samples still arriving 24-34 s in, past the remote lease |
| many_participants_late_joiner | twelve unrelated participants first, the needed one last, against the pool raised to 24 by `CONFIG_RTPS_LIMIT_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` |
| ros2_pub->device_sub_frag_30k | 30 KB samples reassembled from DATA_FRAG over lwIP |
| participant_pool_overflow_recovers | 28 participants against a pool of 24 (and proxy pools of 20), then all leave: discovery keeps going, every one is forgotten after the capped lease, the next publisher is received |
| pool_full_evicts_unused_for_late_joiner | the same 28 against 24, but they STAY, and the publisher the device needs joins last: it can only get in by the eviction of a participant none of the device's endpoints match (`RTPS_SPDP_EVICT_UNUSED_PARTICIPANTS`, oldest first, one per announcement period, each logged); its samples must arrive |
| ros2_client->device_service | the typed service server, FastDDS and CycloneDDS: the embedded build answering the inline-QoS form and the in-payload `{guid, seq}` form |
| netem_loss_service_fastdds, netem_loss_service_cyclone | 15 % loss + 10 ms jitter towards the device and ~14 % loss on the way back (an ingress drop on the TAP): one `ros2 service call` per RMW, which sends a single request and waits - a lost request or reply is repaired by the reliable writers on each side, not retried by the client |
| espp_host_pub->device_sub, device_pub->espp_host_sub, espp_host_frag_30k->device_sub | the host engine against the embedded one, including its DATA_FRAG path |
| ros2_pub->device_sub_frag_30k (wire summary) | tcpdump on the tap during the case: frames per 30 KB sample, fragments, multicast, the biggest burst (FastDDS 42 frames within 20 ms, CycloneDDS 23: why the RX ring is 64) |
| netem_loss_reliable | 15 % loss + 10 ms jitter on the link (`tc netem`): the reliable reader still delivers |
| netem_reorder_reliable, netem_duplicate_reliable, netem_corrupt_reliable | 30 % reordering, 30 % duplication, 2 % corruption on the link: the host engine's numbered samples reach the device's reliable reader every one exactly once and in order |
| netem_reorder_best_effort, netem_duplicate_best_effort | the same against the best-effort reader: samples may be lost, but none arrives twice or out of order (RTPS 2.5 8.4.12.1, `rtps_best_effort_order` on the host) |
| netem_delay_discovery | 300 ms (+-50) delay from before discovery: a ROS 2 publisher is still discovered, matched and delivered over a 600 ms round trip |
| netem_loss_device_pub | 15 % loss towards the device: the device's reliable writer repairs for the host subscriber against lost ACKNACKs |
| device_heap_stable, device_no_crash | free heap at the end within 96 KB of the start (two buffers grow once to the largest sample, 30 KB each); no panic / abort / assert |

The **soak** suite (`./run_qemu.sh <image> soak`, the second CI job on the same
firmware build) runs 20 cycles of churn instead: each cycle six ROS 2
participants and two host-engine ones come, match the device's endpoints every
way they can (reliable and best-effort in, the device's publishers out, a 30 KB
sample, a service call under either RMW) and go, FastDDS and CycloneDDS alternating, with the
previous cycle's leases running out part way into the next. After a warm-up
cycle (whatever grows once to the largest sample grows there) the free heap at
the end must be within 8 KB of the baseline, the low-water mark within 96 KB and stepping down (by 1 KB or more) in at most five cycles,
and the last cycle must still have delivered (`soak_heap_no_leak`,
`soak_min_heap_stable`, `soak_still_delivering`), and every cycle's service
call, Fast DDS and Cyclone alternating, must have been answered, one miss
tolerated over the run (`soak_service_calls`). The per-cycle heap is printed.

The **memory** suite (`./run_qemu.sh <image> memory`, the third CI job) puts
numbers on the footprint instead of asserting on it. The firmware logs the
`sizeof` of the engine's compile-time-sized objects (the Domain, a Participant,
each writer and reader kind, the facade participant: the limits profile and the
capacity knobs decide them, and the Domain that holds them is allocated from the
heap at `start()`), and the heap split the way the ESP32 is - internal RAM,
where every task stack and lwIP buffer must live, against PSRAM - at boot, after
`start()` and after the endpoints exist. The suite then adds one thing at a time
on the host and reads the device's heap after each: a remote participant with
nothing the device subscribes to, a second one, one whose publisher matches the
device's reliable subscriber, a service call, one 30 KB sample, and finally all
peers gone. CI builds the firmware once per limits profile (embedded, host,
host_large: `sdkconfig.profile.*.defaults`, each build with its own `sdkconfig`
- IDF applies `SDKCONFIG_DEFAULTS` only when it creates one, so a build
directory that already has a `sdkconfig` silently keeps its old profile) and
runs the suite on each - the embedded one with the host-driven steps, the
others for their `sizeof` and stage markers (`MEMORY_STAGES_ONLY=1`) - and
`qemu/memory_report.py` merges the suite's `memory.json`
with `idf.py size-components --format json2` (the rtps library's flash, IRAM
and DRAM) into the job's step summary, the `rtps-memory` artifact and, on a
pull request from this repository, one comment on the pull request that every
run updates in place. Costs are
negative deltas of free heap; fragmentation adds noise of a few hundred bytes,
and the largest free internal block is reported alongside the free total
because that is what a task stack or a 30 KB buffer actually needs. Read it
knowing that the engine preallocates: a remote participant or a match costs
tens of bytes of heap because its slot was paid for at `start()` (the Domain,
75 KB on the embedded profile, goes to PSRAM; the 40 KB `start()` takes from
internal RAM is tasks and sockets), so the per-object costs live in the
`sizeof` and stage tables, while the low-water mark column shows the transients
a service call or a 30 KB sample causes on top.

What only this can show: the embedded pools and knobs as built, lwIP's socket
and reassembly behaviour, FreeRTOS priorities for the engine's threads,
delivery under injected loss, heap over a run on the target allocator, and a
panic with a backtrace when something goes wrong. Not performance: QEMU is not
cycle-accurate, so the cases assert on delivery and health, never on rates.

## Not covered (yet): two fragmenting writers on one reader

A reader has one fragment-reassembly slot (`Reader::newFragment`), and any
fragment of a sample other than the one in progress evicts it. Fragments from
writers the reader never matched are ignored (`frag_slot_isolation`), but two
MATCHED writers that both fragment samples to one reader - two cameras on one
topic - still evict each other mid-sample, and under reliability each ACKNACK
round resends whole samples. A per-writer slot costs a reassembly buffer per
matched writer and is left for a follow-up.

## Not covered (yet): a multi-homed peer

A FastDDS or CycloneDDS peer on several networks announces one unicast locator
per interface, and espp must pick the one on its own network (`locator_subnet`
checks the selection logic on canned announcements). Reproducing it live needs
a peer address espp cannot reach, which a single shared network namespace cannot
provide: every local address is reachable there. The design would be a second
Docker network, the ROS 2 side attached to both and espp to one, so the ROS 2
participant announces a locator on the network espp is not on; that is a
two-container `run.sh` and is left for a follow-up.

## Notes

- The engine probes participant ids for free unicast ports (binding with reuse
  disabled), so multiple espp participants can run on one host and the start
  order does not matter.
- Received datagrams are dispatched via `espp::SocketReactor` onto a shared
  worker pool; there is no dedicated per-socket receive task and no engine-owned
  packet queue.
