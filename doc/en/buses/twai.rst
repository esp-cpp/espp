TWAI (CAN 2.0) APIs
*******************

The `Twai` class provides an idiomatic C++ interface to the ESP TWAI (Two-Wire
Automotive Interface, i.e. CAN 2.0) peripheral. It wraps the modern node-based
ESP-IDF driver (`esp_driver_twai`) and takes care of creating an on-chip TWAI
node, registering the driver's ISR event callbacks, and marshaling received
frames (and optional error / state-change events) from ISR context into a
task-context callback via an internal FreeRTOS queue and an ``espp::Task``.

Because the driver's ``on_rx_done`` / ``on_error`` / ``on_state_change``
callbacks run in ISR context, the `Twai` class copies each event into a FreeRTOS
queue from the ISR (``xQueueSendFromISR``) and drains that queue from an internal
task. Your ``on_receive``, ``on_error``, and ``on_state_change`` callbacks are
therefore always invoked from task context - never from the ISR - so they may
safely call blocking or non-IRAM-safe APIs. No lock is held while your callback
runs.

The class supports the classic CAN 2.0 frame format (up to 8 data bytes). CAN-FD
is a possible future extension; the underlying driver and frame types support
it, but this wrapper intentionally keeps to classic CAN for a small, clean
surface.

The example uses ``Mode::LOOPBACK`` (internal loopback + self-test) so it runs on
a bare devkit with no CAN transceiver and no other node on the bus. To talk to a
real bus, use ``Mode::NORMAL`` with a 3.3V CAN transceiver (e.g. SN65HVD230)
wired to the TX/RX GPIOs and at least one other node present to acknowledge
frames.

Transmit result
---------------

``transmit(message, ec)`` waits for the controller to finish with the frame and
reports the outcome through its ``std::error_code`` parameter:
``std::errc::timed_out`` when nothing on the bus acknowledged the frame within
the timeout (with the default ``Config::tx_retry_count = -1`` the controller
keeps retransmitting in the meantime, the standard CAN behaviour), or
``std::errc::io_error`` when the controller gave up on it -- the retries of a
bounded ``tx_retry_count`` were exhausted, a bit error, or arbitration lost
(``Config::on_error`` carries the reason). A frame for which ``transmit()``
returns ``true`` was acknowledged by another node in ``Mode::NORMAL``
(``Mode::LOOPBACK`` waives the acknowledgement). ``tx_retry_count = 0`` makes the
controller try once and drop the frame if that fails, for time-critical data
that must not arrive late.

.. ------------------------------- Example -------------------------------------

.. toctree::

   twai_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/twai.inc
