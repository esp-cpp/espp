# Thread Pool Component

The `ThreadPool` component provides a reusable pool of worker tasks for executing queued jobs asynchronously.

It is implemented with `espp::Task` workers and `std::condition_variable` synchronization.

## Features

- Configurable worker count
- Bounded or unbounded queue
- Optional blocking submit mode for backpressure
- Graceful stop (drains queued jobs)
- Thread-safe stats for submitted / executed / rejected jobs
