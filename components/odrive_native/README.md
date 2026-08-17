# ODrive Native (Fibre endpoint) Protocol Component

[![Badge](https://components.espressif.com/components/espp/odrive_native/badge.svg)](https://components.espressif.com/components/espp/odrive_native)

`espp::OdriveNative` implements a transport-agnostic server for the **ODrive
legacy native (Fibre endpoint) binary protocol** (firmware <= 0.5.x), as used
over the USB vendor interface where each bulk transfer carries exactly one
packet. It parses one inbound request packet and produces one response packet;
it performs no I/O itself (the caller does USB/UART transport).

Applications register typed properties from dotted paths (mirroring
`espp::OdriveAscii`). Endpoint ids are assigned sequentially starting at 1
(endpoint 0 is the JSON descriptor blob), and the compact JSON descriptor and
its CRC are finalized lazily. This lets a legacy `odrivetool` / `fibre-python`
client auto-discover the object tree and perform typed get/set.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ODrive Native (Fibre endpoint) Protocol Component](#odrive-native-fibre-endpoint-protocol-component)
  - [Features](#features)
  - [API](#api)
  - [Protocol](#protocol)
  - [Example](#example)
  - [Notes](#notes)

<!-- markdown-toc end -->

## Features

- **Transport-agnostic**: one packet in via `process_bytes`, one response packet out
- **Typed property registry**: `register_float_property`, and
  `_int8_/_uint8_/_int16_/_uint16_/_int32_/_uint32_/_int64_/_uint64_/_bool_`
  variants, each taking a getter and optional setter (no exceptions; uses
  `std::error_code`)
- **Auto-discovery**: builds the endpoint-0 JSON descriptor + `json_crc` so a
  legacy Fibre client can enumerate the tree
- **No hardware dependencies**: integrates via `std::function` callbacks
- **Thread-safe**: internal locking for the registry; user getters/setters are
  never invoked while a lock is held (snapshot then call)
- **Host-buildable wire core**: the CRC/pack/codec/JSON/dispatch logic lives in
  `detail::OdriveNativeCore`, which builds with just the standard library

## API

Key class: `espp::OdriveNative`
- `process_bytes(std::span<const uint8_t>) -> std::vector<uint8_t>` (one packet in, one out)
- Register properties: `register_float_property`, `register_int32_property`,
  `register_uint32_property`, `register_bool_property`, and the other integer
  width variants
- `finalize()`, `json()`, `json_crc()` inspect the generated descriptor

See header [`include/odrive_native.hpp`](./include/odrive_native.hpp) and
[`PROTOCOL.md`](./PROTOCOL.md) for details.

## Protocol

The authoritative wire specification (packet format, CRC, endpoint dispatch,
type codecs, and JSON schema) is documented in [`PROTOCOL.md`](./PROTOCOL.md).

## Example

A scripted example is provided in [`example`](./example) and is built by CI. It
registers a few simulated-motor properties and feeds crafted packets through
`process_bytes`, logging the responses.

## Notes

- This component implements the **properties** (primitive get/set) surface of the
  legacy protocol; functions / endpoint refs are not implemented yet.
- Wiring to a concrete USB device stack is a later phase; this component is
  purely the protocol server.
