Thread Pool APIs
****************

ThreadPool
----------

The `ThreadPool` component provides a reusable pool of worker tasks for
executing queued jobs asynchronously. Workers are implemented as
:cpp:class:`espp::Task` instances and pull work from an internal job queue
(backed by ``std::deque``) whose maximum size is optionally enforced by
:cpp:member:`espp::ThreadPool::Config::max_queue_size`. Submissions can either
reject immediately when the queue is full or block until space is available,
depending on configuration.

Code examples for the thread pool API are provided in the `thread_pool` example
folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   thread_pool_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/thread_pool.inc
