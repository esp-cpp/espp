Dispatcher
**********

The `Dispatcher` routes framed messages from one byte stream to per-module
handlers, letting several independent protocols share a single USB vendor / CDC
/ socket / UART link. Rather than run a separate ``StreamParser`` per protocol
over the same bytes (each re-buffering the whole stream and needing its own
reset-on-overflow bookkeeping), a Dispatcher parses the
:doc:`../stream_frame/index` stream once and hands each complete frame to the
handler registered for its ``module`` id.

Module id
---------

The frame's ``module`` byte (0..255) is the routing key — a full byte, so up to
256 protocols can coexist on one stream. The message/transaction ``type`` and
the request/reply direction (``flags``) travel with the frame and are handed to
the module's handler untouched; the Dispatcher does not interpret them. espp's
own protocols and examples use these ids by default:

=========  ==============================================
Module id  Protocol
=========  ==============================================
0          OTA
1          Core-dump example crash trigger (example only)
2          BLDC haptics
3          Telemetry
4          crash dump
5          CAN bridge
6          MCP266 console
0xF0-0xFE  reserved (meta)
0xFF       capability discovery
=========  ==============================================

A device-side dispatcher registers the modules it serves; frames for an
unregistered module are silently ignored. A protocol's replies use the **same**
module as its requests (the reply/direction lives in the frame's ``flags``, not
the module), so both directions route to the one registered handler — use
``frame.is_reply()`` to tell them apart. In practice a device only *receives*
requests (it *sends* the replies), so its handler normally sees requests only.
Application code may assign any unused module id to its own protocol; nothing is
hard-wired to a specific service. The ids in the table are **defaults**: the
module id is only a routing key, and every espp service takes its id from
``Config::module`` (used for both the requests it accepts and the replies it
sends), while each example module keeps its id in one named constant. The
defaults are what the hosted web consoles and the ``espp_ota`` CLI look for, so
a device that moves a service off its default must also tell its host tooling
the new id — see :doc:`custom_modules`.

.. ------------------------------- Example -------------------------------------

The example multiplexes two toy protocols over one in-memory byte stream (the
same ``feed()`` / ``register_module()`` pattern applies to a USB vendor / CDC /
socket / UART receive path):

.. literalinclude:: ../../../components/dispatcher/example/main/dispatcher_example.cpp
   :language: cpp
   :start-after: //! [dispatcher example]
   :end-before: //! [dispatcher example]

The codec and dispatcher are also exposed to Python (``espp.stream_frame`` and
``espp.Dispatcher``); see ``python/dispatcher.py`` and ``python/dispatcher_test.py``.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/dispatcher.inc
.. include-build-file:: inc/dispatcher_worker.inc
