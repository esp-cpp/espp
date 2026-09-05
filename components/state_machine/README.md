# Hierarchichal Finite State Machine (HFSM) Component

[![Badge](https://components.espressif.com/components/espp/state_machine/badge.svg)](https://components.espressif.com/components/espp/state_machine)

The `state_machine` component is a light wrapper around the
[webgme-hfsm](https://github.com/finger563/webgme-hfsm) static generated code.
It is designed to be used as the component that one or more specific hfsms
(manually written or generated from webgme-hfsm) can depend on.

Note: This is a generic HFSM implementation - it should be used with generated
code or a manually written state machine, as it provides no functionality on its
own.

## Generating a state machine

`state_machine` is the runtime; the machines themselves are generated
from a model by [webgme-hfsm][repo]. You can model and generate one
entirely in the browser — no install, no server — with the
[**HFSM Playground**][playground], or from the command line:

```sh
npx -p webgme-hfsm hfsm-gen my_machine.json -o generated
```

The example wires that into its CMake, so its C++ is generated at build
time from [`Complex.json`](example/main/Complex.json) rather than
checked in — see [the example README](example/README.md).

[repo]: https://github.com/finger563/webgme-hfsm
[playground]: https://finger563.github.io/webgme-hfsm/

## Example

This example shows an example of running the below HFSM on an ESP32 in a
real-world scenario (e.g. spawning events from one or more threads and running
the HFSM in its own thread) as well as in a test-bench scenario (e.g. running a
CLI to manually spawn events and trace the execution). For more information, see
[webgme-hfsm](https://github.com/finger563/webgme-hfsm).

![hfsm](https://user-images.githubusercontent.com/213467/230950083-d4d8a483-31a7-43ac-8822-b1e28d552984.png)

