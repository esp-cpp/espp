Crash Reporting (Core Dump) APIs
********************************

.. toctree::
    :maxdepth: 1

    coredump

The `CoreDump` component provides idiomatic access to the ESP-IDF flash core
dump — crash detection, a ready-to-print crash report (reset reason, crashed
task, PC, backtrace, decode hint), and raw image access — plus a
transport-agnostic stream service and a browser web app (WebUSB / Web Serial)
for inspecting, downloading and erasing core dumps.
