# CANopen (CiA 301) Client Component

[![Badge](https://components.espressif.com/components/espp/canopen/badge.svg)](https://components.espressif.com/components/espp/canopen)

The `CanopenClient` class provides a lightweight, standards-based CANopen
(CiA 301) client / master for talking to a CANopen server node - for example a
Basicmicro MCP236/MCP266 motor controller - over a classic CAN 2.0 bus. The
`Ds402Drive` class layers the CiA 402 (DS402) drive profile on top of it:
statusword state-machine decoding, the enable-operation sequence, fault reset,
mode selection, and profile-velocity / profile-position motion helpers.

Implemented CiA 301 services (deliberately slim):

* **NMT master** commands (COB-ID `0x000`): start / stop / pre-operational /
  reset node / reset communication, addressed to one node or all nodes.
* **Heartbeat / boot-up consumption** (COB-ID `0x700` + node id): the NMT state
  of every producing node is cached (with an optional callback).
* **SDO client** (COB-IDs `0x600`/`0x580` + node id): expedited upload
  (read) and download (write) of 1/2/4-byte objects with typed
  `read_u8..read_i32` / `write_u8..write_i32` wrappers, segmented upload for
  strings (e.g. manufacturer device name `0x1008`) with toggle-bit handling,
  and abort-code parsing with human-readable messages.
* **PDO helpers**: RPDO transmit (pack + send on a COB-ID) and TPDO reception
  dispatch via per-COB-ID callbacks.
* **SYNC** (COB-ID `0x080`) transmission.

The component is **transport-agnostic**: it transmits by invoking a
user-provided `send` function with a plain `espp::detail::CanFrame`, and the
application feeds received frames to `process_frame()`. The `CanFrame` struct
mirrors `espp::Twai::Message` field-for-field, so wiring it to the `espp/twai`
component is a two-line conversion (see the example) - but any CAN transport
(external SPI CAN controller, USB-CAN bridge, ...) works just as well.

SDO transactions are blocking with a configurable timeout; `process_frame()`
must be called from a different task than the one performing SDO transfers
(automatic with `espp::Twai`, whose `on_receive` runs in its own task).

The wire core (`include/detail/canopen_core.hpp`) is host-buildable pure C++20
with no ESP dependencies, and is covered by golden-frame unit tests in
`test/canopen_host_test.cpp`.

## Example

The [example](./example) uses an `espp::Twai` transport to NMT-start a node,
read its identity and device type via SDO, and - if the device implements
CiA 402 - switch it to profile velocity mode, enable operation, run a gentle
velocity ramp, and stop.
