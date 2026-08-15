# RTPS Example

This example brings up an `espp::RtpsParticipant` on an **ESP32-Ethernet-Kit** and
demonstrates every typed API the `rtps` component offers, interoperable
with FastDDS / ROS 2 over the standard RTPS wire protocol.

It demonstrates:

- Ethernet bring-up (DHCP **server** on `192.168.4.1/24`, so a directly-attached
  PC gets an address) and starting a participant on the interface's IPv4 address
- a typed `Publisher<StringMsg>` / `Subscriber<StringMsg>` pair (reliable pub/sub)
  that pairs with the FastDDS host peer in [`pc/host_pubsub.cpp`](pc/host_pubsub.cpp)
- a typed **service server** (`/add_two_ints`) and **action server**
  (`/fibonacci`) the device hosts — a ROS 2 client can drive them directly with
  `ros2 service call` / `ros2 action send_goal` (no manual CDR; reflectable
  `AddReq`/`AddResp`, `FibGoal`/`FibSeq` structs)
- a typed **service client** + **action client** that call a peer's
  `/peer_add_two_ints` / `/peer_fib` (run a ROS 2 / rclpy server for those names
  to see a full round-trip; otherwise the calls simply time out, still exercising
  the client API)

All of the RMI/AMI code is compiled out when `RTPS_ENABLE_RPC` is disabled.

## How to use example

### Configure

```bash
idf.py menuconfig
```

Under **RTPS Example Configuration**:

| Option | Description |
|---|---|
| `RTPS_EXAMPLE_ANNOUNCE_PERIOD_MS` | Period of the outgoing publisher (default 1500 ms). |
| `RTPS_EXAMPLE_SECOND_PARTICIPANT` | Additively bring up a second, self-testing participant that calls the device's own `/add_two_ints` + `/fibonacci` for a full on-device round-trip (default off; roughly doubles the RTPS engine RAM). |

Under **RTPS (rtps)** you can also toggle the limits profile, dynamic
storage, DATA_FRAG fragmentation, and the RPC (services + actions) layer.

### Build and Flash

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port. Connect the board's Ethernet port to a PC
(or a switch) so the participant has a network.

### Talk to it

- **Pub/sub host peer** (FastDDS): build and run [`pc/host_pubsub.cpp`](pc/)
  against the board's topics.
- **ROS 2**: with `example_interfaces` installed and on the same network/domain:
  ```bash
  ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 35}"
  ros2 action send_goal -f /fibonacci example_interfaces/action/Fibonacci "{order: 5}"
  ```

## Expected Output

The monitor logs Ethernet link-up + the assigned IP, then `tx`/`rx` lines for the
publisher/subscriber and `service '/add_two_ints' + action '/fibonacci' ready`.
Each ROS 2 call logs the handled request/goal (e.g. `service add_two_ints: 7 + 35
= 42`, `action fibonacci(5) done`). With the second participant enabled, `[self-test]`
lines report `PASS`/`FAIL` for the local round-trips.
