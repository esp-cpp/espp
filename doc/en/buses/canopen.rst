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

USB &lt;-&gt; CAN bridge
-------------------

The ``can_bridge_example`` (``components/canopen/can_bridge_example``) turns an
ESP32-S3 into a WebUSB / Web Serial CAN interface: it bridges the `Twai`
(CAN 2.0) controller to the host over USB using the ``stream_frame`` framing and
an :doc:`../dispatcher/index` (module id 5), so the hosted
`CAN console <https://esp-cpp.github.io/espp/apps/can_console.html>`_ web app can
send frames (as a bus master) and inspect the bus (streaming every received
frame, optionally in passive listen-only mode) directly from a Chromium browser.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/canopen_client.inc
.. include-build-file:: inc/ds402.inc
.. include-build-file:: inc/canopen_core.inc
