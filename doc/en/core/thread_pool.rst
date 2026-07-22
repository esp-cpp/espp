Thread Pool APIs
****************

ThreadPool
----------

The `ThreadPool` component provides a reusable pool of worker tasks for
executing queued jobs asynchronously. Workers are implemented as
:cpp:class:`espp::Task` instances and communicate through an optionally bounded
:cpp:class:`std::deque`. Submissions can either reject immediately when the
queue is full or block until space is available, depending on configuration.

Code examples for the thread pool API are provided in the `thread_pool` example
folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   thread_pool_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/thread_pool.inc
