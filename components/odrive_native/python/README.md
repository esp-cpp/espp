# espp_odrive — Python client for the ODrive native (Fibre endpoint) protocol

A clean, ergonomic, **odrivetool-equivalent** Python client for the ODrive
legacy **native** (Fibre endpoint) binary protocol — a from-scratch
re-implementation of the documented wire format in
[`../PROTOCOL.md`](../PROTOCOL.md).

It depends only on the Python **standard library** plus **`pyserial`** (for the
serial backend, imported lazily). There is **no dependency on the
`odrive`/`fibre` pip package** — that is the whole point: you fully own this
code.

> **Shipped with the espp Python package**: this directory is the single source
> of truth, and the [espp wheel](https://pypi.org/project/espp/) ships it as the
> top-level `espp_odrive` package (also reachable as `espp.odrive`) — so
> `pip install espp[serial]` gives you both the bound C++ protocol **server**
> (`espp.OdriveNative`) and this **client**, with no path tricks. See
> `python/odrive_native_test.py` at the repo root, which loops the two against
> each other in-process.

## Install / requirements

- Python 3.8+
- `pyserial` (only needed for the serial transport)

```bash
python3 -m venv .venv && .venv/bin/pip install pyserial
```

## Usage

```python
from espp_odrive import connect

dev = connect("/dev/ttyUSB0")          # downloads endpoint-0 JSON, builds the tree

print("vbus:", dev.vbus_voltage)       # typed read  (float)
print("serial: 0x%X" % dev.serial_number)

dev.axis0.controller.input_pos = 3.14  # typed write (nested objects as attributes)
print(dev.axis0.controller.input_pos)  # read back

# Dotted-path helpers:
dev.set("axis0.controller.config.vel_limit", 42.5)
print(dev.get("axis0.controller.config.vel_limit"))

dev.dump()                             # pretty-print the whole tree with live values
dev.close()
```

`find()` scans available serial ports and returns the first ODrive that answers
(or `None`):

```python
from espp_odrive import find
dev = find(timeout=5.0)
```

### ASCII protocol (optional, separate)

A thin, self-contained helper for the ODrive **ASCII** protocol (`r/w/p/v/f`
text lines) — unrelated to the binary native protocol above:

```python
from espp_odrive import OdriveAscii
a = OdriveAscii("/dev/ttyUSB0")
a.position(0, 3.14)                # p 0 3.14 0 0
pos, vel = a.feedback(0)           # f 0
vbus = float(a.read("vbus_voltage"))
```

## Package layout

| File | Responsibility |
|------|----------------|
| `espp_odrive/crc.py`       | CRC8 / CRC16 (non-reflected, MSB-first) + constants |
| `espp_odrive/protocol.py`  | Packet build/parse, little-endian type codecs |
| `espp_odrive/transport.py` | `Transport` interface + serial **stream framing** backend (`SerialStreamTransport`, `StreamDeframer`, `stream_frame`) |
| `espp_odrive/device.py`    | `Channel` (seq + request/response), object tree (`RemoteObject`/`RemoteProperty`), `connect`/`find`/`Device` |
| `espp_odrive/ascii.py`     | Optional minimal ASCII-protocol helper |

### Transports (serial now, USB later)

The stack talks to the device through a `Transport`, which moves whole
**packets**:

```python
class Transport(ABC):
    def send_packet(self, packet: bytes) -> None: ...
    def read_packet(self, timeout: float) -> bytes | None: ...
```

- `SerialStreamTransport` implements the UART **stream framing**
  (`[0xAA][len][crc8][packet][crc16 big-endian]`) with resynchronizing
  deframing, since a raw serial line has no packet structure.
- **USB seam:** over USB each bulk transfer already *is* one packet, so a future
  USB backend just implements `send_packet`/`read_packet` with no framing and is
  passed to `connect_transport(transport)`. Nothing above the transport changes.

## Wire notes (implemented exactly)

- **CRC16**: poly `0x3d65`, non-reflected MSB-first. Init `0x1337` for stream
  framing and the endpoint-0 packet trailer; init **`PROTOCOL_VERSION=1`** for
  the endpoint `json_crc` canary. Golden: `crc16("123456789", 0x1337)=0xaa01`.
- **CRC8** (stream header only): poly `0x37`, init `0x42`.
- **Packet** (LE): `[seq u16][endpoint u16 (bit15=expect response)][output_len u16][payload][trailer u16]`;
  response `[seq|0x8000 u16][data...]`. Endpoint 0 = chunked JSON read (payload =
  u32 LE offset; empty response when offset ≥ len).
- Sequence numbers increment mod `0x7fff` with bit `0x80` hardwired to 1 (to
  avoid clashing with the ASCII protocol), mirroring the reference client.

## Test / verify

`run.sh` runs the CRC self-test plus an **end-to-end** interop test: it builds
the C++ device shim (`../interop/odrive_native_interop_device.cpp`), spawns it on
a PTY, connects **this** client, enumerates the tree, and read/write-verifies
`vbus_voltage`, `serial_number`, `axis0.controller.input_pos`, and
`axis0.controller.config.vel_limit`.

```bash
./run.sh        # uses ./.venv if present, else creates one with pyserial
```

Expected tail:

```
[test] ALL END-TO-END ASSERTIONS PASSED (espp_odrive client <-> espp device)
==================== SUMMARY ====================
RESULT: PASS
```
