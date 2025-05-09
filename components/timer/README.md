# Timer Component

[![Badge](https://components.espressif.com/components/espp/timer/badge.svg)](https://components.espressif.com/components/espp/timer)

The `timer` component provides a few different types of timers to cover standard
periodic execution as well as periodic / sporadic exectuion which requires
high-resolution timing.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Timer Component](#timer-component)
  - [Timer](#timer)
  - [High-Resolution Timer](#high-resolution-timer)
  - [Example](#example)

<!-- markdown-toc end -->

## Timer

The `Timer` component provides a cross-platform API for executing callback
functions with a specified period. The timer can be started, stopped, and
restarted, and the timer can have an optional initial delay before the first
callback is executed. The timer can be configured to run once or repeatedly.

The timer API is implemented using the `Task` component, and the timer callback
is executed in the context of the timer task.

## High-Resolution Timer

The `HighResolutionTimer` component provides an esp-idf specific API to create
managed high resolution timer objects using the esp_timer API. The timer can be
started, stopped, and restarted, and it can be configured as a one-shot timer
or a periodic timer.

## Example

The [example](./example) shows some various different ways of starting and
stopping timers, as well as some different examples of ways the timers can be
configured, such as repeating or one-shot, and how to use the timer's callback
function.
