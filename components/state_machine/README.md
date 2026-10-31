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
npx -y -p webgme-hfsm@^1.8.0 hfsm-gen my_machine.json -o generated
```

(`-y` skips npx's install prompt for non-interactive/CI use; the `@^1.8.0`
pin matches the example's CMake so generation is reproducible.)

### Generating from your own CMake

This component provides `espp_generate_hfsm()`, so any component can turn a
model into C++ at configure time:

```cmake
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
  espp_generate_hfsm(
    MODEL "${CMAKE_CURRENT_LIST_DIR}/my_machine.json"
    OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/hfsm"
    SOURCES_VAR hfsm_srcs
    INCLUDE_DIR_VAR hfsm_inc)
endif()

idf_component_register(SRCS "main.cpp" ${hfsm_srcs}
                       INCLUDE_DIRS "." ${hfsm_inc})
```

The guard is needed because ESP-IDF does not load a component's
`project_include.cmake` during its early requirements pass — and that pass
does not read `SRCS` either, so the empty variables are what it expects.

It regenerates whenever the model changes, finds the generated file names
rather than making you name them, and leaves out the shared runtime this
component already provides (pass `WITH_SUPPORT` if you want the generator's
copies instead). `NAMESPACE` overrides the model's own.

Requires **node >= 18**; `npx` fetches the generator. Set
`-DESPP_HFSM_GEN_COMMAND="node;/path/to/webgme-hfsm/bin/hfsm-gen.js"` to use a
local checkout, which is also how to build offline.

The example uses exactly this, so its C++ is generated at build time from
[`Complex.json`](example/main/Complex.json) rather than checked in — see
[the example README](example/README.md).

[repo]: https://github.com/finger563/webgme-hfsm
[playground]: https://finger563.github.io/webgme-hfsm/

## Example

This example shows an example of running the below HFSM on an ESP32 in a
real-world scenario (e.g. spawning events from one or more threads and running
the HFSM in its own thread) as well as in a test-bench scenario (e.g. running a
CLI to manually spawn events and trace the execution). For more information, see
[webgme-hfsm](https://github.com/finger563/webgme-hfsm).

![hfsm](https://user-images.githubusercontent.com/213467/230950083-d4d8a483-31a7-43ac-8822-b1e28d552984.png)

