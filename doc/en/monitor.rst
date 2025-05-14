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

API Reference
-------------

.. include-build-file:: inc/task_monitor.inc
