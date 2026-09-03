# MCP266 CANopen Example

This example demonstrates how to use the `espp::Mcp266` component to drive a
Basicmicro MCP266 (RoboClaw-family) motor controller over **CANopen**. It:

1. brings up an `espp::Twai` transport and an `espp::CanopenClient`, wiring the
   received frames into the client,
2. NMT-starts the node and clears any latched CiA 402 faults,
3. reads the main battery voltage and board temperature,
4. configures the M1 position loop (widening the position clamp and seeding a
   non-zero P gain — required once per boot), and
5. runs a small profile-position sequence on M1, reporting arrival at each
   target.

See the [component README](../README.md) for the device specifics (the
manufacturer command-object mirror, the position-PID field-order quirk, and the
velocity-control limitation).

## Requirements

- An ESP target with a TWAI (CAN) controller.
- A 3.3 V CAN transceiver (e.g. SN65HVD230) wired to the configured TX/RX
  GPIOs, on a properly terminated bus.
- A Basicmicro MCP266 configured for CANopen (CAN pins, bit rate, and node id
  set in Basicmicro Motion Studio), with an encoder and a tuned velocity PID.

## Hardware

Edit the `tx_gpio`, `rx_gpio`, `baudrate`, and `node_id` in
`main/mcp266_example.cpp` to match your board and MCP266 configuration.

## Build

Build the project and flash it to the board, then run monitor to view the
serial output:

```sh
idf.py -p PORT flash monitor
```

Replace PORT with the serial port of your board.
