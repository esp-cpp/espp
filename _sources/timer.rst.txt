Timer APIs
**********

Timer
-----

The `Timer` component provides a cross-platform API for executing callback
functions with a specified period. The timer can be started, stopped, and
restarted, and the timer can have an optional initial delay before the first
callback is executed. The timer can be configured to run once or repeatedly.

The timer API is implemented using the `Task` component, and the timer callback
is executed in the context of the timer task.

Code examples for the task API are provided in the `timer` example folder.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/timer.inc
