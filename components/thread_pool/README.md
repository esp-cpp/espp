# Thread Pool Component

[![Badge](https://components.espressif.com/components/espp/thread_pool/badge.svg)](https://components.espressif.com/components/espp/thread_pool)

The `ThreadPool` component provides a reusable pool of worker tasks for executing queued jobs asynchronously.

It is implemented with `espp::Task` workers and `std::condition_variable` synchronization.

## Features

- Configurable worker count
- Bounded or unbounded queue
- Optional blocking submit mode for backpressure
- Manual `start()` / `stop()` control; `start()` returns `true` if all workers launched successfully (or the pool was already running), `false` if any worker failed to start and the pool was rolled back to stopped state
- Graceful stop (stops workers; queued jobs may not be executed)
- Thread-safe stats for submitted / executed / rejected jobs (total and per band)
- **Priority bands** (`espp::QosBand`: `Critical` / `High` / `Normal` / `Low`):
  `submit(job, band)` / `try_submit(job, band)` enqueue into one FIFO queue per
  band and workers always pop the most urgent non-empty band first. The
  band-less `submit(job)` overload uses `Normal`, so existing code is
  unaffected.
- **Aging (anti-starvation)**: a queued job whose wait exceeds
  `Config::aging_threshold` (default 100 ms) is promoted up one band (to the
  back of the next band's queue), so a busy high band cannot starve lower
  bands indefinitely. Set to 0 for strict band priority.
- **Per-band workers** (opt-in via `Config::band_worker_counts`): band *k* gets
  its own workers at `Config::band_task_priorities[k]`, each servicing bands
  0..k (its own band and every more urgent band). On ESP the priorities are
  FreeRTOS task priorities and are always applied; on host platforms
  (Linux/macOS) they map to `SCHED_FIFO` real-time priorities but are **only
  applied when `Config::band_workers_realtime` is set** — by default the
  workers run at the OS default scheduling and band ordering is enforced at
  the queue level only. `SCHED_FIFO` on Linux requires `CAP_SYS_NICE` or an
  `RLIMIT_RTPRIO` allowance (and a spinning job can starve the system — see
  the `Config` docs); without permission the workers fall back gracefully to
  default scheduling with a one-time warning.
