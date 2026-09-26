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
same stream. It answers on :doc:`dispatcher <../dispatcher/dispatcher>`
module id 4 by default; ``Config::module`` moves an instance (requests and
replies alike), but the hosted console looks for 4 until told otherwise.

The hosted `espp Core Dump Console
<https://esp-cpp.github.io/espp/apps/coredump_console.html>`_ web app speaks
the protocol over **WebUSB** (vendor interface) or **Web Serial** (CDC, where
it doubles as a serial monitor): crash summary, chunked ``core.elf``
download, client-side nearest-symbol backtrace resolution against your local
app ELF, and erase.

The ``coredump`` component also ships an idf.py extension (``idf_ext.py``) and
a pure-Python host tool (``components/coredump/python/espp_coredump``), giving
every project that uses it an ``idf.py coredump-usb`` action that pulls the
stored core dump off the device over USB and decodes it against the app ELF it
just built, in one step -- the counterpart of ESP-IDF's ``coredump-info`` /
``coredump-debug``, with options:

.. code-block:: sh

    pip install pyusb esp-coredump   # once
    idf.py coredump-usb              # builds, then downloads + decodes the core dump
    idf.py coredump-usb --gdb        # ... or opens GDB on the core file
    idf.py coredump-usb --summary    # just the crash report stored on the device
    idf.py coredump-usb --erase      # decode (or --summary), then erase the stored dump

idf.py loads a component's extension only from trusted sources (ESP-IDF, the
project's components, ``EXTRA_COMPONENT_DIRS``, ``espressif/`` registry
components); a registry install of ``espp/coredump`` needs
``IDF_EXTENSION_ALLOW_UNTRUSTED=1``, or the espp wheel installed in the IDF
Python environment (its ``idf_extension`` entry point is loaded without a trust
check). Plain ``coredump-usb`` / ``coredump-usb-debug`` CMake targets remain as
a fallback. All of this is the host half only: the firmware must store core
dumps to flash (``CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH`` and a ``coredump``
partition) and serve a ``CoreDumpService`` on a USB vendor interface, as the
example does. The tool also runs directly (``python -m espp_coredump summary``
/ ``download`` / ``debug build/<app>.elf [--gdb]`` / ``erase``) -- see
``components/coredump/python/README.md``.

.. ------------------------------- Example -------------------------------------

.. toctree::

   coredump_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/coredump.inc
.. include-build-file:: inc/coredump_service.inc
