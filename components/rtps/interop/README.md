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
| ros2_pub->espp_sub_liveliness, espp_pub->ros2_sub_liveliness | samples keep arriving 22 s in, i.e. past a remote participant lease (CycloneDDS: 10 s). Only samples after that point count (`rtps_interop_sub ... count_after_s`, `ros2_count_subscriber.py`), so a peer whose SPDP re-announcements were not delivered - and whose lease therefore expired, dropping every matched endpoint - fails. CycloneDDS addresses those re-announcements to readerId ENTITYID_UNKNOWN; the FastDDS pass is the control |
| many_participants_late_joiner | twelve unrelated ROS 2 participants are on the domain before espp starts, then the publisher espp needs joins last and must still be matched. Guards the remote-participant pool (`SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`, 32 on the host profile built here, 10 on the embedded profile - raise it via menuconfig for a domain this busy): a participant beyond the pool is ignored silently |
| cyclone: ... | the ROS 2 pub/sub pairs (reliable and best-effort, both directions) and the two cases above again on `rmw_cyclonedds_cpp`. Services are not covered: rmw_cyclonedds_cpp puts a `{guid, seq}` request header in the payload instead of `related_sample_identity` |

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
