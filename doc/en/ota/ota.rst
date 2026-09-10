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

Command line: build → OTA
-------------------------

The ``ota`` component ships a ``project_include.cmake`` and a pure-Python host
tool (``components/ota/python/espp_ota``), so any project using it can build and
OTA-flash over USB in one step — the OTA counterpart to ``idf.py flash``::

    pip install pyusb      # once (needs a libusb backend)
    idf.py ota-usb        # builds the app, then OTAs it over USB

The tool draws a live progress bar (percent, size, transfer speed, ETA) and
colorizes its output. Because ``idf.py`` captures the target's output, the bar is
drawn straight to the controlling terminal so it still animates in place:

.. image:: https://github.com/user-attachments/assets/a042481a-2964-4b06-9109-bb2dcb4e355b
   :alt: espp_ota flashing an image over USB via idf.py ota-usb
   :width: 100%

.. image:: https://github.com/user-attachments/assets/da8e1b71-c65f-4ecc-9223-e232f8591ceb
   :alt: espp_ota reporting a completed OTA over USB
   :width: 100%

For full control (a specific serial, chunk size, discovery probe) run it directly
with ``python -m espp_ota flash build/<app>.bin`` — see
``components/ota/python/README.md``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   ota_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/ota.inc
.. include-build-file:: inc/ota_stream_protocol.inc
