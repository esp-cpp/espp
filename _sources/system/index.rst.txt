System APIs
***********

.. toctree::
    :maxdepth: 1

    system

The `System` component reports what a device is and how it is doing —
chip, ESP-IDF version, application description, partitions and OTA state,
reset reason, uptime, MAC, memory sizes, CPU frequency and heap — and
controls restarts (a plain reboot, or a reboot into the ROM bootloader's
download mode), as plain C++ APIs and as a transport-agnostic stream service
with a browser web app (WebUSB / Web Serial). The :doc:`monitor
<../core/monitor>` component's ``MonitorService`` complements it with live
heap and task statistics.
