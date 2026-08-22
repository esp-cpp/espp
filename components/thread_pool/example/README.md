# Thread Pool Example

This example is a self-contained test suite that exercises the full public API
of `espp::ThreadPool` across a range of usage patterns, from basic single-pool
operation to more advanced concurrent and multi-pool scenarios.

| # | Test | APIs covered |
|---|---|---|
| 1 | Lifecycle | `start()`, `stop()`, `is_running()`, `worker_count()` |
| 2 | Normal dispatch | `submit()`, `queue_size()`, `stats()` |
| 3 | Bounded-queue rejection | `try_submit()` with deterministic barrier |
| 4 | Blocking submit | `submit()` with `block_on_submit_when_full` |
| 5 | Post-stop rejection | `submit()` after `stop()` |
| 6 | Concurrent lifecycle | multi-thread `start()` / `stop()` stress |
| 7 | Concurrent submission | multi-thread `submit()` + `try_submit()` |
| 8 | Chained pools | a job in `pool_a` submitting work into `pool_b` |
| 9 | Self-submit | a running job submitting back to its own pool |
| 10 | Priority bands | `submit(job, QosBand)` — queued `Critical` jobs overtake queued `Low` jobs; per-band `stats()` counters |

Each test logs individual `PASS` / `FAIL` results inline. At the end of the run
a summary is printed:

```
==================== Results ====================
  PASS  lifecycle: start/stop/is_running/worker_count
  PASS  submit: normal dispatch + queue_size + stats
  ...
=================================================
10/10 tests passed
All tests passed!
```

The final summary line (`N/N tests passed` / `N test(s) FAILED`) is the key
indicator of overall correctness and is suitable for CI log parsing.

## Build

From this directory, run:

```bash
idf.py set-target esp32
idf.py build flash monitor
```
