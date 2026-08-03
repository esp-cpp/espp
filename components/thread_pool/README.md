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
- Thread-safe stats for submitted / executed / rejected jobs
