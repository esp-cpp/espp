# RTPS interop harness (FastDDS, CycloneDDS / ROS 2)

The Phase 0 safety net from `../REFACTOR_PLAN.md`: every refactor phase must keep
this matrix green.

## Run

```bash
cd components/rtps/interop
./run.sh
```

Requires docker. One container (`ros:jazzy-ros-base` = FastDDS + rmw_fastrtps +
`ros2` CLI, plus rmw_cyclonedds_cpp) runs everything in a single network namespace, so RTPS multicast works
unconditionally. The repo is bind-mounted and copied to a container-local tree
before building (espp is installed into a container-local staging prefix and the
pc tests `find_package` it from there), so your host build artifacts are never
touched.

## Matrix

| test | what it proves |
|---|---|
| build | the engine + espp lib build on linux |
| golden | wire-format bytes unchanged (see `pc/tests/rtps_golden.cpp`) |
| locator_subnet | of the unicast locators a peer on several networks announces (SEDP and SPDP), the one on our subnet is kept (see `pc/tests/rtps_locator_subnet.cpp`) |
| loopback | espp<->espp discovery + delivery in one process |
| espp_pub->ros2_echo | espp reliable writer -> ROS 2 subscriber (`rt/chatter`, `std_msgs::msg::dds_::String_`) |
| ros2_pub->espp_sub | ROS 2 publisher -> espp reliable reader |
| espp_be_pub->ros2_be_echo | best-effort pairing |
| ros2_pub->espp_sub_liveliness, espp_pub->ros2_sub_liveliness | samples keep arriving 22 s in, i.e. past a remote participant lease (CycloneDDS: 10 s). Only samples after that point count (`rtps_interop_sub ... count_after_s`, `ros2_count_subscriber.py`), so a peer whose SPDP re-announcements were not delivered - and whose lease therefore expired, dropping every matched endpoint - fails. CycloneDDS addresses those re-announcements to readerId ENTITYID_UNKNOWN; the FastDDS pass is the control. Only the espp-publisher leg detects a dropped peer: a HEARTBEAT that reaches a matched espp reader also refreshes the sending participant's liveliness (`MessageReceiver::processHeartbeat`), so a CycloneDDS *writer* keeps its participant alive through its heartbeats even when its SPDP re-announcements are lost, while an espp writer only gets ACKNACKs back, which do not refresh it, and the peer expires |
| many_participants_late_joiner | twelve unrelated ROS 2 participants are on the domain before espp starts, then the publisher espp needs joins last and must still be matched. Guards the remote-participant pool (`SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`, 32 on the host profile built here, 10 on the embedded profile - raise it via menuconfig for a domain this busy): a participant beyond the pool is ignored silently |
| cyclone: ... | the ROS 2 pub/sub pairs (reliable and best-effort, both directions) and the two cases above again on `rmw_cyclonedds_cpp`. Services are not covered: rmw_cyclonedds_cpp puts a `{guid, seq}` request header in the payload instead of `related_sample_identity` |

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
| ros2_client->device_service | the typed service server (FastDDS rmw) |
| espp_host_pub->device_sub, device_pub->espp_host_sub, espp_host_frag_30k->device_sub | the host engine against the embedded one, including its DATA_FRAG path |
| ros2_pub->device_sub_frag_30k (wire summary) | tcpdump on the tap during the case: frames per 30 KB sample, fragments, multicast, the biggest burst (FastDDS 42 frames within 20 ms, CycloneDDS 23: why the RX ring is 64) |
| netem_loss_reliable | 15 % loss + 10 ms jitter on the link (`tc netem`): the reliable reader still delivers |
| device_heap_stable, device_no_crash | free heap at the end within 96 KB of the start (two buffers grow once to the largest sample, 30 KB each); no panic / abort / assert |

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
