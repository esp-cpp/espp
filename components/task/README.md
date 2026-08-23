# Task Component

[![Badge](https://components.espressif.com/components/espp/task/badge.svg)](https://components.espressif.com/components/espp/task)

The `Task` component provides a cross-platform API around `std::thread` with
some additional configuration of stack size, priority, and core affinity for
FreeRTOS / ESP, as well as providing a callback API which enables interruptible
sleeps and termination of the task.

It also supports firing off syncrhonous (blocking) and asynchronous
(non-blocking) functions in separate threads, with the option of configuring the
core id and other esp-specific paramters.

## Priority semantics per platform

- **ESP (FreeRTOS)**: `BaseConfig::priority` is the FreeRTOS task priority
  (clamped to `configMAX_PRIORITIES - 1`) and is always applied.
- **Host platforms (Linux / macOS / Windows)**: the priority is stored, but only
  applied to the OS thread when `BaseConfig::host_realtime` is set (default
  `false`, preserving the historical behavior of running at the OS default
  scheduling). With the opt-in:
  - **Linux / macOS**: priority ≥ 1 is mapped linearly onto the `SCHED_FIFO`
    real-time priority range (priority 0 resets to the default `SCHED_OTHER`
    scheduler). On Linux this requires `CAP_SYS_NICE` or an `RLIMIT_RTPRIO`
    allowance, and delivers hard preemption on `PREEMPT_RT` kernels; without
    permission the task falls back gracefully to default scheduling with a
    one-time warning. **Beware:** a `SCHED_FIFO` thread that spins can starve
    the rest of the system.
  - **Windows**: best-effort mapping onto `SetThreadPriority()` classes
    (NORMAL / ABOVE_NORMAL / HIGHEST / TIME_CRITICAL).

## Example

The [example](./example) shows some various different ways of starting and
stopping tasks, as well as examples of how to wait, block, exit early, and run
long-running computations within tasks using the `task` component.

