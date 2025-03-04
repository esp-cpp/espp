Monitoring APIs
***************

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
