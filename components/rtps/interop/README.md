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
| loopback | espp<->espp discovery + delivery in one process |
| espp_pub->ros2_echo | espp reliable writer -> ROS 2 subscriber (`rt/chatter`, `std_msgs::msg::dds_::String_`) |
| ros2_pub->espp_sub | ROS 2 publisher -> espp reliable reader |
| espp_be_pub->ros2_be_echo | best-effort pairing |
| ros2_pub->espp_sub_liveliness, espp_pub->ros2_sub_liveliness | samples keep arriving 22 s in, i.e. past a remote participant lease (CycloneDDS: 10 s). Only samples after that point count (`rtps_interop_sub ... count_after_s`, `ros2_count_subscriber.py`), so a peer whose SPDP re-announcements were not delivered - and whose lease therefore expired, dropping every matched endpoint - fails. CycloneDDS addresses those re-announcements to readerId ENTITYID_UNKNOWN; the FastDDS pass is the control. Only the espp-publisher leg detects a dropped peer: a HEARTBEAT that reaches a matched espp reader also refreshes the sending participant's liveliness (`MessageReceiver::processHeartbeat`), so a CycloneDDS *writer* keeps its participant alive through its heartbeats even when its SPDP re-announcements are lost, while an espp writer only gets ACKNACKs back, which do not refresh it, and the peer expires |
| many_participants_late_joiner | twelve unrelated ROS 2 participants are on the domain before espp starts, then the publisher espp needs joins last and must still be matched. Guards the remote-participant pool (`SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`, 32 on the host profile built here, 10 on the embedded profile - raise it via menuconfig for a domain this busy): with the pool full, a newly announcing participant evicts the oldest one none of espp's endpoints match (`RTPS_SPDP_EVICT_UNUSED_PARTICIPANTS`, on by default, one eviction per announcement period, logged); with it off a participant beyond the pool is ignored silently |
| ros2_pub_churn->espp_sub_unmatch | a ROS 2 node creates, uses and destroys a publisher five times in 25 s and exits (`ros2_churn_publisher.py`): the espp subscriber must see five matched->unmatched transitions, each after samples arrived, within 40 s (`rtps_interop_churn`, polling `RtpsParticipant::matched_writers()`). Only a peer's dispose can do that; a writer dropped on lease expiry alone (20 s Fast DDS, 10 s Cyclone DDS) would give at most two |
| espp_service_server<-ros2_client, espp_service_client->ros2_server | AddTwoInts both ways against a live ROS 2 client (`ros2 service call`) and server (rclpy) |
| espp_action_server<-ros2_client, espp_action_client->ros2_server | Fibonacci both ways: send_goal, feedback, deferred get_result |
| espp_service_client_x8->ros2_server | eight requests in flight at once from one client, each with its own operands: every reply must land on the request it answers (`rtps_service_interop_client ... 8`) |
| two_espp_clients->ros2_server | two espp clients call one ROS 2 server at the same time: a client must take only the replies addressed to it |
| espp_service_client->late_ros2_server | the client is up and calling before any server exists; the ROS 2 server joins 8 s later and the call must still succeed |
| espp_trigger_server<-ros2_client | `std_srvs/srv/Trigger`: the smallest request a ROS 2 client can send (an empty message, one placeholder byte) and a 48000-character reply in a single DATA (`rtps_trigger_interop_server`) |
| espp_action_server<-ros2_cancel, espp_action_server<-ros2_reject | goal paths `ros2 action send_goal` cannot drive (`ros2_fibonacci_client.py`): a goal cancelled at its first feedback ends CANCELED with the partial sequence; a goal the server refuses (order 0) is reported rejected |
| cyclone: ... | every ROS 2 case above again on `rmw_cyclonedds_cpp`: the pub/sub pairs (reliable and best-effort, both directions), liveliness, the busy domain, the publisher churn, and the four service/action cases. rmw_cyclonedds_cpp correlates a request with its reply through a `{guid, seq}` header inside the payload rather than the `related_sample_identity` inline QoS rmw_fastrtps_cpp uses; espp detects the peer's vendor and answers in its form (`pc/tests/rtps_cyclonedds_rpc_header.cpp` pins the codec) |
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

What only this can show: the embedded pools and knobs as built, lwIP's socket
and reassembly behaviour, FreeRTOS priorities for the engine's threads,
delivery under injected loss, heap over a run on the target allocator, and a
panic with a backtrace when something goes wrong. Not performance: QEMU is not
cycle-accurate, so the cases assert on delivery and health, never on rates.

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
