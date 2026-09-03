Motor Controller Interface
**************************

The `motor_controller` component holds the shared interface used by the
dual-channel motor-controller drivers that model the same MCP236 / MCP266
hardware over different transports:

- :cpp:enum:`espp::MotorAxis`, the ``M1`` / ``M2`` channel selector, and
- the :cpp:concept:`espp::MotorController` concept — the common command / read
  surface (``drive_duty`` / ``drive_speed`` by axis, ``read_encoder`` /
  ``read_speed``, ``reset_estop``, ``read_main_battery_voltage``,
  ``read_temperature``).

Keeping the contract in a small, dependency-light component lets the two drivers
share it without depending on each other: :doc:`basicmicro` (packet serial) and
:doc:`mcp266` (CANopen) each ``static_assert`` that they satisfy
``MotorController``, so generic code can command either transport by axis.

.. code-block:: cpp

   // works for either espp::Basicmicro or espp::Mcp266
   bool ramp(espp::MotorController auto &mc, std::error_code &ec) {
     return mc.drive_duty(espp::MotorAxis::M1, 4096, ec);
   }

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/motor_controller.inc
