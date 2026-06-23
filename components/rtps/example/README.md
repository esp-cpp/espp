# RTPS Example

This example now acts as a two-node RTPS smoke test for ESP targets on the same
Wi-Fi network.

It demonstrates:

* Wi-Fi STA setup for host-network RTPS traffic
* standard RTPS UDP port calculation for each participant
* SPDP participant discovery between two boards
* SEDP endpoint discovery for request/response topics
* CDR little-endian serialization for `std_msgs/msg/UInt32`-style payloads
* best-effort inter-node request/response sample exchange

The component's long-term goal is ROS 2 interoperability over DDS/RTPS. This
example focuses on proving cross-board discovery and user-data delivery using
the current scaffold.

## How to use example

### Configure two boards

Build one board as the **initiator** and the other as the **responder**.

For both boards:

1. Set the same `RTPS domain ID`, `Topic prefix`, `WiFi SSID`, and `WiFi password`.
2. Give each board a unique `RTPS participant ID`.
3. Give each board a distinct `Participant node name` or keep the role-specific defaults.
4. If you want to exercise multicast user data, enable `Use best-effort user-data multicast`
   on both boards and keep the same request/response multicast groups on each node.

Fresh example configurations now default to:

* initiator: participant ID `1`, node name `espp_rtps_initiator`
* responder: participant ID `2`, node name `espp_rtps_responder`

If you are reusing an older build directory or `sdkconfig`, rerun `idf.py menuconfig`
or delete the stale generated config so the old shared defaults (`participant ID = 1`,
`node name = espp_rtps_node`) do not persist on both boards.

For one board only:

1. Select `RTPS Example Configuration -> Example Role -> Initiator`

For the other board:

1. Select `RTPS Example Configuration -> Example Role -> Responder`

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```sh
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

## Example Output

The initiator waits until it discovers the responder's endpoints, then publishes
incrementing values on `<topic-prefix>/request`. The responder logs each
received request and echoes the same value back on `<topic-prefix>/response`.

Expected signs of success:

* both boards report Wi-Fi connection and their local IP address
* both boards log RTPS participant and endpoint discovery
* the initiator logs `Published request N` followed by `Received response N`
* the responder logs `Received request N, sending response`

When multicast user data is enabled, discovery still uses the normal RTPS
metatraffic sockets, but request samples are published to the configured request
multicast group and response samples are published to the configured response
multicast group. Each node only joins the group for the topic it subscribes to.
