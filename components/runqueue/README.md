# RunQueue Asynchronous Execution Component

[![Badge](https://components.espressif.com/components/espp/runqueue/badge.svg)](https://components.espressif.com/components/espp/runqueue)

The `RunQueue` component provides an implementation of a task runqueue which can
be used to run functions asynchronously and with optional relative priorities.
The runqueue itself is implemented as a priority queue of functions that is
handled within its own task. This task is just a standard `espp::Task`, so it
can be configured with custom task priority and core id (as well as stack size
and other parameters).

## Example

The [example](./example) shows how you can use the `espp::RunQueue` to schedule
functions to run asynchronously with priority ordering. You can configure the
RunQueue task so that you have different RunQueue tasks for different groups of
function priorities as well as so that you can have a RunQueue on each core if
you like.

