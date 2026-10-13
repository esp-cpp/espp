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
``std::errc::io_error`` when the controller gave up on it after the configured
retry limit (``Config::tx_retry_count``, 0 means one attempt) was exhausted by
a missing acknowledgement, a bit error, or arbitration lost (``Config::on_error``
carries the reason) -- or ``std::errc::timed_out`` when, with ``tx_retry_count =
-1`` (retransmit until acknowledged, the standard CAN behaviour), nothing
acknowledged the frame within the timeout. A frame for which ``transmit()``
returns ``true`` was acknowledged by another node in ``Mode::NORMAL``
(``Mode::LOOPBACK`` waives the acknowledgement). The default single attempt suits
time-critical data that must not arrive late; set a retry count, or -1, when a
frame must get through.

Pending transmissions
---------------------

``transmit()`` is synchronous, so once it returns nothing of yours is normally
left with the controller. The exception is a frame whose completion wait timed
out: with ``tx_retry_count = -1`` and nothing acknowledging, the controller keeps
retransmitting it. The ESP-IDF driver offers no abort for that (disabling the
node only pauses the transmission; re-enabling resumes it), so ``espp::Twai``
provides ``flush(ec, timeout_ms)`` -- wait until the controller has no pending
transmission, e.g. before a stop that must not be followed by anything older --
and ``abort_pending(ec)`` -- drop the pending transmission(s) by deleting and
re-creating the node (same configuration, callbacks and filter; the receive task
and everything already received are untouched). ``Config::auto_abort_on_timeout``
(default ``true``) makes a ``transmit()`` that timed out drop its frame that way;
with it off the frame stays with the controller and the next ``transmit()`` first
waits for it (up to its own timeout, failing with ``std::errc::timed_out`` while
it is still pending).

.. ------------------------------- Example -------------------------------------

.. toctree::

   twai_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/twai.inc
