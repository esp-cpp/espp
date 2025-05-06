# Monitor Component

The `monitor` component provides utilities for monitoring various aspects of the
system.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Monitor Component](#monitor-component)
  - [Task Monitor](#task-monitor)
  - [Example](#example)

<!-- markdown-toc end -->

## Task Monitor

The task monitor provides the ability to use the FreeRTOS trace facility to
output information about the CPU utilization (%), stack high water mark (bytes),
and priority of all the tasks running on the system.

There is an associated [task-monitor](https://github.com/esp-cpp/task-monitor)
python gui which can parse the output of this component and render it as a chart
or into a table for visualization.

## Example

This example shows how to use the `monitor` component to monitor the executing
tasks.
