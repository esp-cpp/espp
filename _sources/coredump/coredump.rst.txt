Core Dump (Crash Reporting)
***************************

The `CoreDump` class wraps ESP-IDF's flash core dump (``espcoredump`` with
``CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y`` and a ``coredump`` data partition)
in an idiomatic espp API: ``has_core_dump()``, ``summary()`` (the raw
``esp_core_dump_summary_t``), and ``format_report()`` — a ready-to-print text
report with the reset reason, panic reason, crashed task + PC, the raw
backtrace addresses (Xtensa, with a ``(corrupted)`` marker when the on-device
unwind failed) or captured stack dump (RISC-V), and the exact ``addr2line``
command line using the right toolchain prefix for the build target. Abnormal
resets that write no core dump (brownout, interrupt / task watchdog) are
still reported with a short hint. Raw image access (``image_size()``,
``read_image()``, ``erase()``) supports downloading the complete ELF core
dump over any transport; all failures are reported via ``std::error_code``.

The `CoreDumpService` class serves that information over **any byte stream**:
it reuses the espp :doc:`ota <../ota/ota>` component's CRC-32-verified stream
framing (``detail/ota_stream_protocol.hpp``) with message types in a
dedicated range (``GET_SUMMARY`` / ``GET_SIZE`` / ``READ`` / ``ERASE``
requests, ``SUMMARY`` / ``SIZE`` / ``DATA`` / ``OK`` / ``ERROR`` replies).
Construct it with a ``send`` function and feed it received bytes — mounting
it on a USB vendor (WebUSB) callback, a CDC (Web Serial) callback, or a
socket takes a few lines. Unknown frame types are ignored, so the service
coexists with other framed protocols — and, because the parser
resynchronizes on the frame magic, with free-form **console text** — on the
same stream.

The hosted `espp Core Dump Console
<https://esp-cpp.github.io/espp/apps/coredump_console.html>`_ web app speaks
the protocol over **WebUSB** (vendor interface) or **Web Serial** (CDC, where
it doubles as a serial monitor): crash summary, chunked ``core.elf``
download, client-side nearest-symbol backtrace resolution against your local
app ELF, and erase.

.. ------------------------------- Example -------------------------------------

.. toctree::

   coredump_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/coredump.inc
.. include-build-file:: inc/coredump_service.inc
