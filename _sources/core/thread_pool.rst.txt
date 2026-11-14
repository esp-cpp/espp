Thread Pool APIs
****************

ThreadPool
----------

The :cpp:class:`espp::ThreadPool` component provides a reusable pool of worker tasks for
executing queued jobs asynchronously. Workers are implemented as
:cpp:class:`espp::Task` instances and pull work from internal job queues whose
combined maximum size is optionally enforced by
:cpp:member:`espp::ThreadPool::Config::max_queue_size`. Submissions can either
reject immediately when the queue is full or block until space is available,
depending on configuration.

Priority bands
^^^^^^^^^^^^^^

Jobs can be submitted at one of four :cpp:enum:`espp::QosBand` priority bands —
``Critical`` / ``High`` / ``Normal`` / ``Low`` — via
``submit(job, band)`` / ``try_submit(job, band)``. Internally the pool keeps one
FIFO queue per band and workers always pop the most urgent non-empty band
first; the band-less ``submit(job)`` overload uses ``Normal``, so code that does
not use bands behaves exactly as before. Per-band submitted / executed / aged / rejected
counters are reported through :cpp:member:`espp::ThreadPool::Stats`.

To keep a busy high band from starving lower bands, a queued job whose wait
exceeds :cpp:member:`espp::ThreadPool::Config::aging_threshold` (default 100 ms)
is *aged*: promoted up one band, to the back of that band's queue. Setting the
threshold to 0 disables aging (strict band priority).

Per-band workers
^^^^^^^^^^^^^^^^

By default all workers are identical and serve every band. Setting
:cpp:member:`espp::ThreadPool::Config::band_worker_counts` opts into dedicated
per-band workers: band *k* gets its own workers running at
:cpp:member:`espp::ThreadPool::Config::band_task_priorities` [k], each servicing
bands 0..k (its own band and every more urgent band), so a ``Critical`` job
never waits behind more than one in-flight lower-band job. The deepest (least
urgent) configured band's workers service *every* band, so no band is ever
unreachable — even with aging disabled.

On ESP the per-band priorities are FreeRTOS task priorities and are always
applied. On host platforms (Linux / macOS) they map onto ``SCHED_FIFO``
real-time priorities, but are **only applied when**
:cpp:member:`espp::ThreadPool::Config::band_workers_realtime` **is set** — by
default host workers run at the OS default scheduling and band ordering is
enforced at the queue level only. See the :cpp:class:`espp::Task` documentation
for the host real-time scheduling requirements and caveats.

Code examples for the thread pool API are provided in the ``thread_pool`` example
folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   thread_pool_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/thread_pool.inc
.. include-build-file:: inc/qos_band.inc
