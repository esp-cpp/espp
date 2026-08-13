RunQueue APIs
*************

RunQueue
--------

The `RunQueue` component provides an implementation of a task runqueue which can
be used to run functions asynchronously and with optional relative priorities.
The runqueue itself is implemented as a priority queue of functions that is
handled within its own task. This task is just a standard `espp::Task`, so it
can be configured with custom task priority and core id (as well as stack size
and other parameters).

Code examples for the runqueue API are provided in the `runqueue` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   runqueue_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/runqueue.inc
