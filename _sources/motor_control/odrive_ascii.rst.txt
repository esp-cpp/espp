ODrive ASCII Protocol Component
================================

Overview
--------

``espp::OdriveAscii`` implements a minimal, dependency-free server for the
ODrive-compatible ASCII protocol. It parses incoming bytes into commands and
produces response bytes for transmission by the caller. It does not perform any
I/O itself and is transport-agnostic (UART, USB CDC, socket, etc.).

Interactive Web Console
-----------------------

A single-file, dependency-free browser console for this protocol is hosted with
these docs. Because this page is served over HTTPS (a secure context), the
browser `Web Serial API <https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API>`_
works directly here — in a Chromium-based browser (Chrome / Edge / Opera /
Brave), click **Connect** and pick your board's USB-Serial-JTAG / CDC port.

`Open the Web Serial console in a new tab <../apps/odrive_console.html>`_
(the same file also lives in ``components/odrive_ascii/web/`` for offline use).

.. raw:: html

   <iframe src="../apps/odrive_console.html" allow="serial"
           title="ODrive ASCII Web Serial console"
           style="width:100%;height:640px;border:1px solid var(--color-background-border,#ccc);border-radius:8px;margin-top:0.5em"></iframe>

A **WebUSB** variant is also hosted, for boards that expose a dedicated USB
vendor interface (see the ``usb_device`` component). It talks to that vendor
interface directly rather than a serial port:
`Open the WebUSB console in a new tab <../apps/odrive_webusb_console.html>`_.

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
- ``w <path> <value>``: writes a registered property
- ``p <axis> <pos> [vel_ff [torque_ff]]``: position setpoint
- ``v <axis> <vel> [torque_ff]``: velocity setpoint
- ``c <axis> <torque_nm>``: torque (Nm) setpoint
- ``t <axis> <goal_pos_turns>``: trajectory goal position (turns)
- ``f <axis>``: feedback (returns "<pos> <vel>\n")
- ``es <axis> <abs_pos_turns>``: set encoder absolute position (turns)

By default the server matches the ODrive protocol and is **silent** on writes
and setpoint commands (``w``/``p``/``v``/``c``/``t``/``es``): only ``r``/``f``/``help``
respond. Errors are always reported.
Set ``Config::acknowledge_commands = true`` to additionally get an ``OK\n`` reply
on successful writes/setpoints (useful for a custom client, but avoid it when
talking to tools like ``odrivetool`` that expect the silent protocol).

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
