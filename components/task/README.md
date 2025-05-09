# Task Component

[![Badge](https://components.espressif.com/components/espp/task/badge.svg)](https://components.espressif.com/components/espp/task)

The `Task` component provides a cross-platform API around `std::thread` with
some additional configuration of stack size, priority, and core affinity for
FreeRTOS / ESP, as well as providing a callback API which enables interruptible
sleeps and termination of the task.

It also supports firing off syncrhonous (blocking) and asynchronous
(non-blocking) functions in separate threads, with the option of configuring the
core id and other esp-specific paramters.

## Example

The [example](./example) shows some various different ways of starting and
stopping tasks, as well as examples of how to wait, block, exit early, and run
long-running computations within tasks using the `task` component.

