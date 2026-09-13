Task APIs
*********

Task
----

The `Task` component provides a cross-platform API around `std::thread` with
some additional configuration of stack size, priority, and core affinity for
FreeRTOS / ESP, as well as providing a callback API which enables interruptible
sleeps and termination of the task.

It also supports firing off syncrhonous (blocking) and asynchronous
(non-blocking) functions in separate threads, with the option of configuring the
core id and other esp-specific paramters.

Priority semantics per platform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On ESP, :cpp:member:`espp::Task::BaseConfig::priority` is the FreeRTOS task
priority (clamped to ``configMAX_PRIORITIES - 1``) and is always applied. On
host platforms (Linux / macOS / Windows) the priority is stored, but only
applied to the OS thread when :cpp:member:`espp::Task::BaseConfig::host_realtime`
is set (default ``false``, preserving the historical behavior of running at the
OS default scheduling). With the opt-in, Linux and macOS map priority ≥ 1
linearly onto the ``SCHED_FIFO`` real-time priority range (priority 0 resets to
the default ``SCHED_OTHER`` scheduler), and Windows maps it best-effort onto
``SetThreadPriority()`` classes.

.. warning::

   A ``SCHED_FIFO`` thread that spins can starve the rest of the system. On
   Linux, real-time scheduling requires ``CAP_SYS_NICE`` or an
   ``RLIMIT_RTPRIO`` allowance (and delivers hard preemption on ``PREEMPT_RT``
   kernels); without permission the task falls back gracefully to default
   scheduling with a one-time warning.

Code examples for the task API are provided in the `task` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   task_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/task.inc
.. include-build-file:: inc/run_on_core.inc
