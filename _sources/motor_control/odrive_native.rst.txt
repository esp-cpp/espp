ODrive Native (Fibre endpoint) Protocol Component
=================================================

Overview
--------

``espp::OdriveNative`` implements a transport-agnostic server for the ODrive
legacy native (Fibre endpoint) binary protocol (firmware <= 0.5.x), as used over
the USB vendor interface where each bulk transfer carries exactly one packet. It
parses one inbound request packet and produces one response packet; it performs
no I/O itself.

Applications register typed properties from dotted paths (mirroring
``espp::OdriveAscii``). Endpoint ids are assigned sequentially starting at 1
(endpoint 0 is the JSON descriptor blob), and the compact JSON descriptor and its
CRC are finalized lazily. This lets a legacy ``odrivetool`` / ``fibre-python``
client auto-discover the object tree and perform typed get/set.

The CRC / packet packing / type codecs / JSON descriptor / dispatch logic lives
in ``espp::detail::OdriveNativeCore``, a host-buildable wire core that depends
only on the C++ standard library, so the protocol can be unit-tested off-target.

Features
--------

- Transport-agnostic: one packet in via ``process_bytes``, one response packet out
- Typed property registry: ``register_float_property`` plus signed/unsigned 8/16/32/64-bit
  integer and ``bool`` variants (no exceptions; uses ``std::error_code``)
- Auto-discovery: builds the endpoint-0 JSON descriptor and ``json_crc``
- Thread-safe; user getters/setters are never invoked while a lock is held
- No direct hardware dependencies; uses ``std::function`` for DI

Basic Usage
-----------

.. code-block:: cpp

  espp::OdriveNative proto({.log_level = espp::Logger::Verbosity::INFO});
  float vbus = 24.0f, input_pos = 0.0f;
  proto.register_float_property("vbus_voltage", [&]() { return vbus; });
  proto.register_float_property("axis0.controller.input_pos",
                                [&]() { return input_pos; },
                                [&](float v, std::error_code &ec) { input_pos = v; ec.clear(); return true; });

  // One USB bulk transfer == one packet.
  auto resp = proto.process_bytes(std::span<const uint8_t>(rx_buf, rx_len));
  // Transmit resp back over the same transport (empty when no response expected)

Protocol
--------

The authoritative wire specification (packet format, CRC-16 with poly 0x3d65 /
init 0x1337, endpoint dispatch, little-endian type codecs, and the compact JSON
schema) is documented in ``components/odrive_native/PROTOCOL.md``.

Notes
-----

This component implements the property (primitive get/set) surface of the legacy
protocol; functions / endpoint refs are not implemented yet. Wiring to a concrete
USB device stack is handled in a later phase.

.. ------------------------------- Example -------------------------------------

.. toctree::

   odrive_native_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/odrive_native.inc
