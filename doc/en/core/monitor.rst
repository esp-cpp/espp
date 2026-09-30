Monitoring APIs
***************

Heap Monitor
------------

The heap monitor provides some simple utilities for monitoring and printing out
the state of the heap memory in the system. It uses various `heap_caps_get_*`
functions to provide information about a memory region specified by a bitmask of
capabilities defining the region:

* `minimum free bytes`: The minimum free bytes available in the region over the
  lifetime of the region.
* `free bytes`: The current number of free bytes available in the region.
* `allocated bytes`: The current number of allocated bytes in the region.
* `largest free block`: The size of the current largest free block (in bytes) in
  the region. Any mallocs over the size will fail.
* `total size`: The size (in bytes) of the memory region.

It provides some utilities for formatting the output as single line output, CSV
output, or a nice table.

Finally, the class provides some static methods for some common use cases to
quickly get the available memory for various regions as well as easily format
them into csv/table output.

Code examples for the monitor API are provided in the `monitor` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   monitor_example

.. ---------------------------- API Reference ----------------------------------

Heap Monitor API Reference
--------------------------

.. include-build-file:: inc/heap_monitor.inc

Task Monitor
------------

The task monitor provides the ability to use the FreeRTOS trace facility to
output information about the CPU utilization (%), stack high water mark (bytes),
and priority of all the tasks running on the system.

There is an associated `task-monitor <https://github.com/esp-cpp/task-monitor>`_
python gui which can parse the output of this component and render it as a chart
or into a table for visualization.

Code examples for the monitor API are provided in the `monitor` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   monitor_example

.. ---------------------------- API Reference ----------------------------------

Task Monitor API Reference
--------------------------

.. include-build-file:: inc/task_monitor.inc

Monitor Service
---------------

The `MonitorService` class serves the heap-region and task statistics above
over **any byte stream** as a :doc:`dispatcher <../dispatcher/dispatcher>`
module (``espp.monitor`` v1, module id 8 by default; ``Config::module`` moves
an instance and hosts find it through discovery by its protocol id).
``GET_HEAP`` answers with one record per configured heap region
(``Config::heap_regions``, ``MALLOC_CAP_*`` masks; regions the chip does not
have are left out), ``GET_TASKS`` with the ``TaskMonitor`` table (name, CPU %,
stack high-water mark, priority, core — it needs
``CONFIG_FREERTOS_USE_TRACE_FACILITY`` and
``CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS``, else the list is empty), and
``SET_STREAM`` starts a task that sends either or both periodically so a host
can plot them live. The wire codec (``detail/monitor_protocol.hpp``) is
host-buildable and unit-tested (``test/monitor_host_test.cpp``). The hosted
`espp System Console <https://esp-cpp.github.io/espp/apps/system_console.html>`_
web app renders the heap gauges and a live task table; see the
:doc:`system <../system/system>` component's example, which exposes both
services over USB.

Monitor Service API Reference
-----------------------------

.. include-build-file:: inc/monitor_service.inc
