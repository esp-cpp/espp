CANopen (CiA 301) Client APIs
*****************************

The `CanopenClient` class provides a lightweight, standards-based CANopen
(CiA 301) client / master for talking to a CANopen server node - for example a
Basicmicro MCP236/MCP266 motor controller - over a classic CAN 2.0 bus. It
implements NMT master commands, heartbeat / boot-up consumption, an SDO client
(expedited upload/download of 1/2/4-byte objects with typed accessors, plus
segmented upload for strings such as the manufacturer device name ``0x1008``),
RPDO transmit / TPDO reception dispatch, and SYNC transmission.

The `Ds402Drive` class layers the CiA 402 (DS402) drive profile on top of a
`CanopenClient`: it decodes the statusword into the standard power-drive-system
states, walks the enable sequence (Shutdown -> Switch On -> Enable Operation)
with statusword polling and timeout, supports quick stop and fault reset, mode
selection (profile velocity / profile position / homing) verified via the modes
display object, and provides motion helpers such as ``set_target_velocity()``
and ``set_target_position()`` (including the profile-position new-set-point
controlword handshake).

The client is transport-agnostic: it transmits by invoking a user-provided
``send`` function with a plain ``espp::detail::CanFrame``, and the application
feeds received frames to ``process_frame()``. The ``CanFrame`` struct mirrors
``espp::Twai::Message`` field-for-field, so wiring it to the `Twai` component
is a trivial conversion (as the example shows) - but any CAN transport works.

SDO transactions are blocking with a configurable timeout, so
``process_frame()`` must be called from a different task than the one
performing SDO transfers; with `Twai` this is automatically the case since its
``on_receive`` callback runs in the Twai receive task.

.. ------------------------------- Example -------------------------------------

.. toctree::

   canopen_example

USB to CAN bridge
-----------------

The ``can_bridge_example`` (``components/canopen/can_bridge_example``) turns an
ESP32-S3 into a WebUSB / Web Serial CAN interface: it bridges the ``Twai``
(CAN 2.0) controller to the host over USB using the ``stream_frame`` framing and
an :doc:`../dispatcher/index` (module id 5), so the hosted
`CAN bridge console <https://esp-cpp.github.io/espp/apps/can_bridge_console.html>`_ web app can
send frames (as a bus master) and inspect the bus (streaming every received
frame, optionally in passive listen-only mode) directly from a Chromium browser.

For CiA 402 (DS402) drives there is also a purpose-built
`DS402 drive panel <https://esp-cpp.github.io/espp/apps/ds402_panel.html>`_ web
app that talks to the *same* bridge. It layers a CANopen SDO client and the DS402
profile entirely in the browser (the firmware stays a dumb raw-CAN pipe): read
the node identity, walk the power-drive-system state machine (statusword /
controlword with the enable sequence, quick-stop and fault-reset), select the
mode of operation, set target and read actual velocity/position/torque, and
read/write any object dictionary index:subindex (including vendor-specific
objects) with a chosen data type. Because it drives the node over SDO rather than
cyclic PDOs it is a commissioning / bring-up tool; the bus must be in Normal
(not listen-only) mode so the bridge ACKs the node.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/canopen_client.inc
.. include-build-file:: inc/ds402.inc
.. include-build-file:: inc/canopen_core.inc
