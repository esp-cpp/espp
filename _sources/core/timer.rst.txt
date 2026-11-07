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

The timer schedules against an absolute wake-up time (the k-th callback targets
``start + k * period``) rather than sleeping for ``period`` after each callback,
so it does not accumulate drift even when an individual callback runs long or
the scheduler jitters. If a callback overruns the period, subsequent callbacks
run back-to-back to catch up (a rate-limited warning is logged) instead of the
schedule slipping permanently.

Timing and resolution
^^^^^^^^^^^^^^^^^^^^^^^

On ESP / FreeRTOS the timer waits on the scheduler, which can only resolve time
to a single tick (``1 / CONFIG_FREERTOS_HZ`` seconds - e.g. 10 ms at the 100 Hz
default, or 1 ms at 1000 Hz). A period or delay that is shorter than - or within
a couple of ticks of - the tick period cannot be honored accurately: it is
rounded up to a whole number of ticks and can jitter by up to a full tick. The
constructor, :cpp:func:`set_period` and :cpp:func:`start` log a warning when the
requested period/delay is at or near the tick period.

For sub-tick periods or highly accurate periodic work, either raise
``CONFIG_FREERTOS_HZ`` (menuconfig: ``FreeRTOS`` → ``Tick rate (Hz)``) or use the
esp_timer-based ``HighResolutionTimer`` described below.

Code examples for the task API are provided in the `timer` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   timer_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/timer.inc

High Resolution Timer
---------------------

The `HighResolutionTimer` component provides an esp-idf specific API to create
managed high resolution timer objects using the esp_timer API. The timer can be
started, stopped, and restarted, and it can be configured as a one-shot timer
or a periodic timer.

.. ------------------------------- Example -------------------------------------

.. toctree::

   timer_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/high_resolution_timer.inc
