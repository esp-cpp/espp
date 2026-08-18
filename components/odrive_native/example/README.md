# ODrive Native (Fibre endpoint) Example

This example demonstrates how to use the `espp::OdriveNative` component to serve
the ODrive legacy native (Fibre endpoint) binary protocol. It registers a few
simulated-motor properties, then feeds crafted request packets through
`process_bytes` and logs the responses:

1. an endpoint-0 read that returns the auto-generated JSON descriptor,
2. a binary write to `axis0.controller.input_pos`, and
3. a binary read of the same property.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ODrive Native (Fibre endpoint) Example](#odrive-native-fibre-endpoint-example)
  - [Requirements](#requirements)
  - [Build](#build)
  - [Flash and Monitor](#flash-and-monitor)
  - [Notes](#notes)

<!-- markdown-toc end -->

## Requirements

- ESP-IDF installed and `get_idf` available in your shell

## Build

```sh
# From repo root
cd components/odrive_native/example
get_idf
idf.py build
```

## Flash and Monitor

```sh
idf.py flash monitor
```

The example runs a scripted packet sequence and logs the descriptor and the
per-packet responses.

## Notes

- The component is transport-agnostic: this example fabricates packets in code.
  In a real deployment each USB bulk transfer would carry one packet, which you
  pass to `process_bytes`, transmitting the returned response bytes back.
- Wiring to a concrete USB device stack is a later phase.
