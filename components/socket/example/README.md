# Socket Example

This example now acts as both a usage demo and a lightweight regression test for
the `socket` component. It runs a sequence of UDP and TCP scenarios and prints a
pass/fail summary at the end.

The covered scenarios include:

* UDP unicast client/server messaging with scope-based teardown
* UDP request/response with a larger payload
* UDP multicast request/response
* UDP response timeout when no server is listening
* UDP blocked-receive teardown
* TCP unicast client/server messaging with scope-based teardown
* TCP request/response followed by reconnect
* TCP blocked-accept teardown
* TCP connect failure to an unused port

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

The serial log shows each scenario as it starts, a per-scenario pass/fail line,
and a final summary reporting how many scenarios completed successfully.
