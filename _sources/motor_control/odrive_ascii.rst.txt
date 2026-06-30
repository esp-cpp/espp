ODrive ASCII Protocol Component
================================

Overview
--------

``espp::OdriveAscii`` implements a minimal, dependency-free server for the
ODrive-compatible ASCII protocol. It parses incoming bytes into commands and
produces response bytes for transmission by the caller. It does not perform any
I/O itself and is transport-agnostic (UART, USB CDC, socket, etc.).

Features
--------

- Register string-addressable properties with read/write callbacks (no exceptions; uses std::error_code)
- High-rate commands: ``p`` (position), ``v`` (velocity), ``t``/``c`` (torque/current)
- Property access commands: ``r <path>``, ``w <path> <value>``
- Re-entrant, thread-safe; minimal allocations
- No direct hardware dependencies; uses ``std::function`` for DI

Basic Usage
-----------

.. code-block:: cpp

  espp::OdriveAscii proto({.log_level = espp::Logger::Verbosity::INFO});
  float position = 0.0f;
  proto.register_float_property("axis0.encoder.pos_estimate",
                                [&]() { return position; },
                                [&](float v, std::error_code &ec) { position = v; ec.clear(); return true; });
  proto.on_position_command([&](int axis, float pos, std::optional<float> vel_ff,
                                std::optional<float> torque_ff, std::error_code &ec) {
    (void)axis; position = pos; return true;
  });

  // Feed incoming data (from UART, etc.)
  auto resp = proto.process_bytes(std::span<const uint8_t>(rx_buf, rx_len));
  // Transmit resp back over the same transport

Commands
--------

- ``help``: prints brief usage
- ``r <path>``: reads a registered property, returns ``<value>\n``
- ``w <path> <value>``: writes a registered property, returns ``OK\n`` or ``ERR\n``
- ``p <axis> <pos> [vel_ff [torque_ff]]``: position setpoint; returns ``OK\n`` or ``ERR\n``
- ``v <axis> <vel> [torque_ff]``: velocity setpoint
- ``c <axis> <torque_nm>``: torque (Nm) setpoint
- ``t <axis> <goal_pos_turns>``: trajectory goal position (turns)
- ``f <axis>``: feedback (returns "<pos> <vel>\n")
- ``es <axis> <abs_pos_turns>``: set encoder absolute position (turns)

Notes
-----

This is a pragmatic subset designed for easy integration with the Python
``odrive`` package's ASCII endpoint. Extend by registering additional properties
that mirror your controller's configuration/state. The component never touches
hardware or transport layers.

.. ------------------------------- Example -------------------------------------

.. toctree::

   odrive_ascii_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/odrive_ascii.inc
