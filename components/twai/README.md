# TWAI (CAN 2.0) Component

[![Badge](https://components.espressif.com/components/espp/twai/badge.svg)](https://components.espressif.com/components/espp/twai)

The `Twai` class provides an idiomatic C++ interface to the ESP TWAI (Two-Wire
Automotive Interface, i.e. CAN 2.0) peripheral. It wraps the modern node-based
ESP-IDF driver (`esp_driver_twai`) and takes care of creating an on-chip TWAI
node, registering the driver's ISR event callbacks, and marshaling received
frames (and optional error / state-change events) from ISR context into a
task-context callback via an internal FreeRTOS queue and an `espp::Task`.

Because the driver's `on_rx_done` / `on_error` / `on_state_change` callbacks run
in ISR context, the `Twai` class copies each event into a FreeRTOS queue from
the ISR (`xQueueSendFromISR`) and drains that queue from an internal task. Your
`on_receive`, `on_error`, and `on_state_change` callbacks are therefore always
invoked from task context - never from the ISR - so they may safely call
blocking or non-IRAM-safe APIs. No lock is held while your callback runs.

The class supports the classic CAN 2.0 frame format (up to 8 data bytes).
CAN-FD (up to 64 data bytes and bit-rate switching) is a possible future
extension - the underlying driver and frame types support it, but this wrapper
intentionally keeps to classic CAN for a small, clean surface.

## Modes

* `Mode::NORMAL` - transmit, receive, and acknowledge frames. Requires a CAN
  transceiver (e.g. SN65HVD230, TJA1050, MCP2551) and at least one other node on
  the bus to acknowledge frames.
* `Mode::LISTEN_ONLY` - only monitor the bus; never transmit or acknowledge.
  Useful for passive bus monitoring / sniffing.
* `Mode::LOOPBACK` - internal loopback + self-test: the controller receives back
  the frames it transmits and does not require an acknowledgement. This lets the
  node run with **no transceiver** and **no other node**, which is what the
  example uses.

## Hardware / wiring (NORMAL mode)

To talk to a real CAN bus you need a 3.3V CAN transceiver between the ESP TWAI
TX/RX pins and the bus:

```
  ESP32 GPIO(tx) ---> CTX  \
                            SN65HVD230  ==> CANH / CANL (120R terminated bus)
  ESP32 GPIO(rx) <--- CRX  /
```

Terminate the bus with 120Ω resistors at both physical ends. Typical baud rates
are 125 kbit/s, 250 kbit/s, 500 kbit/s (common in automotive), and 1 Mbit/s.

## Example

The [example](./example) uses `Mode::LOOPBACK` so it runs on a bare devkit with
no transceiver: it transmits a few frames, receives them back via the
`on_receive` callback, and asserts the round trip.
