# CANopen Client Example

This example demonstrates the use of the `espp::CanopenClient` and
`espp::Ds402Drive` classes to talk to a CANopen (CiA 301) server node - for
example a Basicmicro MCP236/MCP266 motor controller - over the ESP TWAI
(CAN 2.0) peripheral via the `espp::Twai` class.

It:

* Brings up the TWAI peripheral in `NORMAL` mode and wires its task-context
  `on_receive` callback to `CanopenClient::process_frame()` (the
  transport-agnostic `CanFrame` struct mirrors `espp::Twai::Message`
  field-for-field).
* Sends an **NMT start** to a configurable node id.
* **SDO-reads** the standard identification objects: device type (`0x1000`),
  identity (`0x1018` vendor / product / revision / serial), and the
  manufacturer device name (`0x1008`, a string read via **segmented** SDO
  upload with toggle-bit handling).
* If the device reports the CiA 402 device profile: selects **profile velocity
  mode**, walks the CiA 402 state machine to **Operation Enabled** (with an
  automatic fault reset if needed), runs a gentle velocity ramp up and back
  down while logging the actual velocity, then stops and disables the drive.

## How to use example

### Hardware Required

An ESP chip with a TWAI peripheral (e.g. ESP32 or ESP32-S3), a 3.3V CAN
transceiver (e.g. SN65HVD230, TJA1050, MCP2551) wired between the configured
TX/RX GPIOs and the bus, and a CANopen device on a properly (120Ω) terminated
bus. Update the `node_id`, GPIOs, and baudrate at the top of the example to
match your setup (Basicmicro MCP2xx controllers default to 250 kbit/s).

```
  ESP32 GPIO(tx) ---> CTX  \
                            SN65HVD230  ==> CANH / CANL (120R terminated bus)
  ESP32 GPIO(rx) <--- CRX  /
```

### Build and Flash

```
idf.py set-target esp32
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

```
[CANopen Example/I][0.518]: Starting CANopen (CiA 301) client example!
[CANopen Example/I][0.530]: Sent NMT start to node 1
[CANopen Example/I][0.735]: Device type (0x1000): 0x00020192
[CANopen Example/I][0.740]: Vendor id (0x1018:1): 0x00000123
[CANopen Example/I][0.746]: Product code (0x1018:2): 0x00000266
[CANopen Example/I][0.752]: Revision (0x1018:3): 0x00010000
[CANopen Example/I][0.758]: Serial number (0x1018:4): 0x0000BEEF
[CANopen Example/I][0.770]: Device name (0x1008): 'MCP266 2x60A'
[CANopen Example/I][0.776]: Drive state: Switch on disabled
[Ds402Drive/I][0.850]: enable_operation: starting from state 'Switch on disabled'
[Ds402Drive/I][1.050]: enable_operation: drive is in Operation Enabled
[CANopen Example/I][1.560]: target= 100, actual=  98
...
[CANopen Example/I][6.560]: target=   0, actual=   1
[CANopen Example/I][6.660]: Motion demo complete
[CANopen Example/I][6.665]: CANopen example complete!
```
