# ODrive Native (legacy Fibre endpoint) protocol — implementation spec

Authoritative wire spec for `espp::OdriveNative`, extracted from the ODrive
firmware reference (`fw-v0.5.1`): `Firmware/fibre/python/fibre/protocol.py` and
`Firmware/fibre/cpp/include/fibre/protocol.hpp`.

**Target:** the legacy endpoint protocol (fw ≤ 0.5.x), **packet-based**, as used
over the USB **vendor** interface (one bulk IN + one bulk OUT). Each USB bulk
transfer carries exactly one packet (USB provides the reliability the UART
stream framing otherwise adds). Goal: `odrivetool` (legacy backend)
auto-discovers the object tree and does typed get/set. The newer 0.6+/Pro Fibre
is a different, larger stack and is out of scope for now.

## Constants
- `PROTOCOL_VERSION = 1`
- CRC8:  init `0x42`, poly `0x37`   (only used by the UART *stream* framing)
- CRC16: init `0x1337`, poly `0x3d65`

## CRC algorithm (both widths, **non-reflected, MSB-first, bit-by-bit**)
```
calc_crc(remainder, byte, poly, bitwidth):      # byte in [0,255]
    topbit = 1 << (bitwidth - 1)
    remainder ^= byte << (bitwidth - 8)
    repeat 8 times:
        remainder = (remainder & topbit) ? ((remainder << 1) ^ poly)
                                         :  (remainder << 1)
    return remainder & ((1 << bitwidth) - 1)
# CRC over a buffer: start from init, fold each byte through calc_crc.
```

## Packet format (host ⇄ device, little-endian throughout)
**Request** (host → device):
```
[seq_no      u16 LE]   # MSB (0x8000) clear in requests; client sets bit 0x80, masks 0x7fff
[endpoint_id u16 LE]   # bit15 (0x8000) set => client expects a response; low 15 bits = endpoint #
[output_len  u16 LE]   # number of response bytes the client wants back
[payload     ...   ]   # bytes to WRITE, or the read OFFSET (u32 LE) for endpoint 0; empty for a plain read
[trailer     u16 LE]   # canary: PROTOCOL_VERSION(1) if endpoint#==0, else json_crc
```
**Response** (device → host, emitted only if endpoint_id bit15 was set):
```
[seq_no u16 LE]        # = request seq_no with MSB (0x8000) set
[data   ...    ]       # up to output_len bytes (the value / json chunk); empty for a pure write
```
The server **must ignore** a request whose `trailer` != the expected canary
(PROTOCOL_VERSION for endpoint 0, else `json_crc`) — this is how the client and
server confirm they share the same object model.

## Server dispatch
- **endpoint 0** (the JSON blob): `payload` = offset (u32 LE). Respond with
  `json[offset : offset + min(output_len, 512)]`. `offset >= len(json)` → empty
  response (that is how the client's read loop terminates). trailer == PROTOCOL_VERSION.
- **endpoint N in registry**: if `payload` non-empty → deserialize per type and
  **write** (when writable); if `output_len > 0` → serialize the current value
  (per type, `output_len` bytes) into the response. trailer == `json_crc`.
- **unknown endpoint**: ignore (empty response).

## Type codecs (little-endian)
| type          | size | notes            |
|---------------|------|------------------|
| `bool`        | 1    | 0/1              |
| `int8`/`uint8`| 1    |                  |
| `int16`/`uint16` | 2 |                  |
| `int32`/`uint32` | 4 |                  |
| `int64`/`uint64` | 8 |                  |
| `float`       | 4    | IEEE-754         |
| `endpoint_ref`| 4    | `[endpoint u16][json_crc u16]` |

## JSON descriptor (the endpoint-0 blob)
Compact UTF-8 JSON (**no insignificant whitespace** — `json_crc` is over the
exact bytes). Top level is an **array** of the root object's members. Entries:
- property: `{"name":<str>,"id":<int>,"type":<primitive>,"access":"r"|"rw"|"w"}`
- object:   `{"name":<str>,"type":"object","members":[ ... ]}`
- function: `{"name":<str>,"id":<int>,"type":"function","inputs":[...],"outputs":[...]}`

`json_crc` = `calc_crc16(json_bytes, init=PROTOCOL_VERSION=1)` — the endpoint
canary is seeded with `PROTOCOL_VERSION`, **not** the 0x1337 packet-CRC init. This
matches the fw-v0.5.1 firmware (`endpoints_template.j2`:
`json_crc_ = calc_crc16(PROTOCOL_VERSION, embedded_json, len)`) and the reference
fibre client (`discovery.py`: `calc_crc16(PROTOCOL_VERSION, json_bytes)`). The
server computes it over the bytes it emits; `odrivetool` computes it over the
bytes it reads; they must match byte-for-byte. (Verified by the serial-loopback
interop harness in `interop/`; only the 0x1337 init applies to the UART *stream*
framing CRC16 and the packet trailer of endpoint 0, which is `PROTOCOL_VERSION`.)

## `espp::OdriveNative` (transport-agnostic, mirrors `espp::OdriveAscii`)
- `std::vector<uint8_t> process_bytes(std::span<const uint8_t>)` — one packet in,
  one response packet out (empty if none). The caller performs USB I/O.
- Registration builds a typed endpoint tree (getters/setters via `std::function`,
  `std::error_code`, no exceptions); ids are assigned and the JSON + `json_crc`
  are finalized at build time. Names use dotted paths like
  `axis0.controller.input_pos`, mirroring the ASCII component.

## Verification plan
1. **CRC golden vectors** generated from the exact Python reference (above) — the
   C++ `calc_crc16` must match bit-for-bit.
2. **Packet round-trip** host unit tests: crafted read / write / endpoint-0-read
   requests → exact expected response bytes.
3. **Real interop** (later phase, the true gate): run genuine `fibre-python` /
   `odrivetool` against the host build over a loopback and confirm it enumerates
   the tree and reads/writes endpoints — mirrors how `rtps` is gated against
   FastDDS / ROS 2.
