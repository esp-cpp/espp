# RTPS interop harness (FastDDS / ROS 2)

The Phase 0 safety net from `../REFACTOR_PLAN.md`: every refactor phase must keep
this matrix green.

## Run

```bash
cd components/rtps/interop
./run.sh
```

Requires docker. One container (`ros:jazzy-ros-base` = FastDDS + rmw_fastrtps +
`ros2` CLI) runs everything in a single network namespace, so RTPS multicast works
unconditionally. The repo is bind-mounted and copied to a container-local tree
before building (espp is installed into a container-local staging prefix and the
pc tests `find_package` it from there), so your host build artifacts are never
touched.

## Matrix

| test | what it proves |
|---|---|
| build | the engine + espp lib build on linux |
| golden | wire-format bytes unchanged (see `pc/tests/rtps_golden.cpp`) |
| loopback | espp<->espp discovery + delivery in one process |
| espp_pub->ros2_echo | espp reliable writer -> ROS 2 subscriber (`rt/chatter`, `std_msgs::msg::dds_::String_`) |
| ros2_pub->espp_sub | ROS 2 publisher -> espp reliable reader |
| espp_be_pub->ros2_be_echo | best-effort pairing |

## Notes

- The engine probes participant ids for free unicast ports (binding with reuse
  disabled), so multiple espp participants can run on one host and the start
  order does not matter.
- Received datagrams are dispatched via `espp::SocketReactor` onto a shared
  worker pool; there is no dedicated per-socket receive task and no engine-owned
  packet queue.
