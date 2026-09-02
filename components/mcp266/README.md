# MCP266 CANopen Motor Controller Component

[![Badge](https://components.espressif.com/components/espp/mcp266/badge.svg)](https://components.espressif.com/components/espp/mcp266)

The `Mcp266` class is a dual-channel controller for a Basicmicro **MCP266**
(RoboClaw family) brushed-DC motor driver over **CANopen**. It is layered on
`espp::CanopenClient` (like `espp::Ds402Drive`), so it is transport-agnostic:
the application owns the CAN transport, feeds received frames to the client's
`process_frame()`, and the client's node id selects the MCP266.

Both motor channels (`M1`, `M2`) are driven symmetrically. M2's CiA 402
objects mirror M1's at `+0x800`, handled through `Ds402Drive`'s object offset.

## What works, and what does not

**Position control** uses the standard CiA 402 profile position mode
(`move_to_position`) and is the supported, validated capability. It needs the
position loop configured first (`configure_position_loop`).

**Velocity / duty control is not functional** on the MCP266 firmware tested.
The standard target objects and the manufacturer speed/duty command mirror are
both accepted by the drive but leave the velocity generator idle even with the
drive in Operation Enabled. Supported-drive-modes (`0x6502`) advertises only
the cyclic-sync modes, so velocity likely requires csv mode with cyclic
SYNC/PDO updates, which is undocumented for this device. `drive_speed` /
`drive_duty` are implemented but are currently a no-op for motion; use position
mode.

## Device specifics

The MCP266's control-loop parameters are **not** standard CiA 402 objects. The
MCP mirrors its packet-serial command set into the manufacturer region of the
object dictionary at index `0x2000 + command number` (see
`include/detail/mcp266_core.hpp`, which is host-buildable and unit-tested).
This component uses that to:

* configure the position PID (commands 61-64),
* issue the manufacturer speed/duty commands (32/33, 35/36), and
* read telemetry: main battery (24) and temperature (82).

Two device quirks are handled by `configure_position_loop()`, which must be
called once per boot (the MCP reverts to its EEPROM configuration on power-up):

* The position PID's `MinPos`/`MaxPos` clamp defaults to `[0, 0]`, which forces
  every position target to zero — it is widened here.
* The setter (commands 61/62) uses field order `D, P, I` while the readback
  (63/64) uses `P, I, D`, so a naive read-modify-write of the record would move
  `P` into the `D` slot and zero `P`. The correct field shuffle (and seeding a
  non-zero `P` when the record has none) is done here.

## Example

The [example](./example) brings up an `espp::Twai` transport and an
`espp::CanopenClient`, NMT-starts the node, reads telemetry, configures the M1
position loop, and runs a small profile-position sequence.

## Related components

* `espp/canopen` — the CANopen client and the `Ds402Drive` CiA 402 helper this
  component builds on.
* `espp/basicmicro` — the Basicmicro **packet serial** protocol driver (a
  different transport to the same controller family), including velocity and
  position PID configuration over UART.
