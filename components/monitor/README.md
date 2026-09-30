# Monitor Component

[![Badge](https://components.espressif.com/components/espp/monitor/badge.svg)](https://components.espressif.com/components/espp/monitor)

The `monitor` component provides utilities for monitoring various aspects of the
system.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Monitor Component](#monitor-component)
  - [Task Monitor](#task-monitor)
  - [Monitor Service](#monitor-service)
  - [Example](#example)

<!-- markdown-toc end -->

## Task Monitor

The task monitor provides the ability to use the FreeRTOS trace facility to
output information about the CPU utilization (%), stack high water mark (bytes),
and priority of all the tasks running on the system.

There is an associated [task-monitor](https://github.com/esp-cpp/task-monitor)
python gui which can parse the output of this component and render it as a chart
or into a table for visualization.

## Monitor Service

`espp::MonitorService` (`monitor_service.hpp`) serves the heap-region and task
statistics over any framed byte stream as an `espp::Dispatcher` module
(`espp.monitor` v1, module 8 by default): `GET_HEAP` (one record per configured
`MALLOC_CAP_*` region), `GET_TASKS` (the `TaskMonitor` table; needs
`CONFIG_FREERTOS_USE_TRACE_FACILITY` + `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`)
and `SET_STREAM` (periodic HEAP / TASKS events). The wire codec lives in
`include/detail/monitor_protocol.hpp` (host-buildable, tested by
`test/monitor_host_test.cpp`). The hosted
[system console](https://esp-cpp.github.io/espp/apps/system_console.html) web
app renders heap gauges and a live task table from it; the
[system](../system) component's example exposes it over USB together with
`espp::SystemService`.

## Example

This example shows how to use the `monitor` component to monitor the executing
tasks.
