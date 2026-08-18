# TWAI (CAN 2.0) Example

This example demonstrates the use of the `espp::Twai` class to communicate over
the ESP TWAI (CAN 2.0) peripheral using the modern `esp_driver_twai` node API.

It runs in `Mode::LOOPBACK` (internal loopback + self-test) so it works on a
bare devkit with **no CAN transceiver** and **no second node** on the bus: it
transmits a few frames, receives each of them back via the task-context
`on_receive` callback, logs both TX and RX, and asserts the round trip.

## Switching to a real bus (NORMAL mode)

To talk to a real CAN bus, change the mode to `espp::Twai::Mode::NORMAL` and wire
a 3.3V CAN transceiver (e.g. SN65HVD230, TJA1050, MCP2551) between the ESP TWAI
TX/RX GPIOs and the bus:

```
  ESP32 GPIO(tx) ---> CTX  \
                            SN65HVD230  ==> CANH / CANL (120R terminated bus)
  ESP32 GPIO(rx) <--- CRX  /
```

Terminate the bus with 120Ω resistors at both physical ends, and make sure at
least one other node is present to acknowledge frames (in NORMAL mode a frame is
only considered transmitted once acknowledged). Typical baud rates are 125 kbit/s,
250 kbit/s, 500 kbit/s, and 1 Mbit/s.

## How to use example

### Hardware Required

This example can run on any ESP chip that has a TWAI peripheral (e.g. ESP32 or
ESP32-S3). No external hardware is required in the default loopback mode.

### Build and Flash

```
idf.py set-target esp32
idf.py -p PORT flash monitor
```

(Replace `PORT` with the name of the serial port.)

## Example Output

The example logs each transmitted (`TX:`) and received (`RX:`) frame and then a
`SUCCESS:` line confirming all frames were received back in loopback mode.
