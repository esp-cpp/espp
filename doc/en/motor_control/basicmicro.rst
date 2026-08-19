Basicmicro (MCP / RoboClaw) Motor Controller Component
======================================================

Overview
--------

``espp::Basicmicro`` is a driver for Basicmicro MCP236 / MCP266 (and other
RoboClaw-family) brushed DC motor controllers speaking their packet serial
protocol, typically over UART.

The component is transport-agnostic: it performs no I/O itself and instead
calls user-provided ``write`` / ``read`` functions for each transaction, so it
works over a UART driver, USB CDC, an RS-232 adapter, etc. All wire-format
logic (CRC16, packet building, reply validation, big-endian codecs) lives in
``espp::detail`` inside ``include/detail/basicmicro_core.hpp``, a
host-buildable core that depends only on the C++20 standard library and is
unit-tested off-target.

Features
--------

- Duty-cycle drive (commands 32/33/34) and closed-loop speed drive in
  quadrature pulses per second (35/36/37), with optional acceleration ramps
  (38/39/40)
- Buffered speed / accel / distance motion commands (41-46) plus buffer-state
  readback (47)
- Encoder counts (16/17/78), speeds (18/19/79), reset (20) and encoder-mode
  readback (91)
- Velocity PID get/set with automatic 16.16 fixed-point conversion (28/29,
  55/56)
- Telemetry: firmware version (21), battery voltages (24/25), motor currents
  (49), motor PWMs (48), board temperatures (82/83) and unit status (90)
- Management: write settings to EEPROM (94), E-Stop reset (200)
- No exceptions; all methods report errors via ``std::error_code``
- Thread-safe: each transaction (request write + ACK/reply read) is serialized
  by an internal mutex

Protocol
--------

Per the MCP Series User Manual (section 2.2), write commands send
``[Address, Command, Data..., CRC16]`` and are acknowledged with a single
``0xFF`` byte, while read commands send ``[Address, Command]`` (no CRC) and
reply with the data followed by a CRC16 seeded with the sent address and
command bytes. All multi-byte values are big-endian and the CRC is
CRC-16/XMODEM (poly ``0x1021``, init ``0``). A 10 ms inter-byte gap clears the
controller's packet buffer, so the configured receive timeout (>= 10 ms)
doubles as the error-recovery mechanism.

Basic Usage
-----------

.. code-block:: cpp

  espp::Basicmicro mcp({
      .address = 0x80,
      .write = [](std::span<const uint8_t> data) { /* UART write */ return true; },
      .read = [](std::span<uint8_t> data, std::chrono::milliseconds timeout) -> size_t {
        /* UART read with timeout, return bytes read */ return 0;
      },
  });

  std::error_code ec;
  std::string version;
  mcp.read_firmware_version(version, ec);
  mcp.drive_m1_duty(4096, ec);   // ~12.5% duty
  uint32_t count; uint8_t status;
  mcp.read_encoder_m1(count, status, ec);
  mcp.drive_m1_duty(0, ec);

.. ------------------------------- Example -------------------------------------

.. toctree::

   basicmicro_example.md

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/basicmicro.inc
