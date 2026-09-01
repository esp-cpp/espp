OTA (Over-the-Air Firmware Update)
**********************************

The `Ota` class is a transport-agnostic OTA firmware update engine wrapping
ESP-IDF's ``esp_ota_ops``: a single mutex-serialized update session
(``begin()`` -> ``write()`` ... -> ``finish()`` / ``abort()``) with all
failures reported via ``std::error_code``. It performs no I/O itself — feed it
image bytes from any transport and it streams them into the next OTA app
partition. The first chunk is validated against the ESP image magic byte
(0xE9) and the incoming application descriptor (project name, version, build
date) is extracted, logged and exposed; ``finish()`` runs the full image
validation (including the appended SHA-256) and sets the boot partition, while
the restart is a separate explicit ``restart()`` call.

Rollback helpers (``is_pending_verify()``, ``mark_app_valid()``,
``mark_app_invalid_and_rollback()``) integrate with the bootloader's app
rollback support (``CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE``): an app booted
pending-verify must call ``mark_app_valid()`` after its own health checks or
the bootloader rolls back to the previous image on the next reset.

For OTA over a raw byte stream (such as the
:doc:`usb_device <../buses/usb_cdc>` vendor / WebUSB interface), the header
``detail/ota_stream_protocol.hpp`` provides a host-testable framed protocol —
CRC-32-verified little-endian frames (``BEGIN`` / ``DATA`` / ``FINISH`` /
``ABORT`` and ``OK`` / ``ERROR`` / ``PROGRESS`` replies) with an incremental,
resynchronizing parser and a bounded 4096-byte maximum payload. The hosted
`espp OTA Console <https://esp-cpp.github.io/espp/apps/ota_console.html>`_ web
app speaks this protocol over WebUSB directly from a Chromium browser.

The frame codec itself lives in the reusable :doc:`../stream_frame/index`
component (``detail/ota_stream_protocol.hpp`` re-exports it and layers the OTA
message types on top); to run OTA alongside other protocols (crash-dump, CAN,
...) on one stream, register it as a module with the
:doc:`../dispatcher/index` — the ``ota`` example does exactly this (OTA is
module id 0).

.. ------------------------------- Example -------------------------------------

.. toctree::

   ota_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/ota.inc
.. include-build-file:: inc/ota_stream_protocol.inc
