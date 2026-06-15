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
3. Optionally set distinct `Participant node name` values to make discovery logs easier to read.

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
