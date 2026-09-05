# Hierarchichal Finite State Machine (HFSM) Example

This example shows an example of running the below HFSM on an ESP32 in a
real-world scenario (e.g. spawning events from one or more threads and running
the HFSM in its own thread) as well as in a test-bench scenario (e.g. running a
CLI to manually spawn events and trace the execution). For more information, see
[webgme-hfsm](https://github.com/finger563/webgme-hfsm).

![hfsm](https://user-images.githubusercontent.com/213467/230950083-d4d8a483-31a7-43ac-8822-b1e28d552984.png)

## The state machine is generated, not checked in

The HFSM's C++ is generated from [`main/Complex.json`](main/Complex.json)
every time the example is configured. The model is the source; the C++ is
a build product, like an object file.

This needs **node (>= 18)** on your PATH — nothing else, and nothing to
install by hand. `npx` fetches the generator on demand:

```
idf.py build      # generates main/Complex.json -> build/.../hfsm/, then builds
```

Editing the model regenerates on the next build; you do not need to clean.

Only the machine itself is generated. The shared runtime it builds on
— `state_base.hpp`, the history states, `magic_enum.hpp` — comes from
this component, via the generator's `--no-support` flag; espp's copies
are the ones the rest of the codebase is built against.

To edit the machine, open it in the **[HFSM Playground][playground]** — a
browser-based editor and code generator, no install and no server. Load
`main/Complex.json` with **Open file…**, or jump straight to this
example's machine:

[**Open this example's HFSM in the playground**][this-model]

Save the edited model back over `main/Complex.json` and rebuild.

If you are working on the generator itself, point the build at your
checkout instead of npx:

```
idf.py -DHFSM_GEN_COMMAND="node;/path/to/webgme-hfsm/bin/hfsm-gen.js" build
```

The same flag is the way to build without network access.

[playground]: https://finger563.github.io/webgme-hfsm/
[this-model]: https://finger563.github.io/webgme-hfsm/?example=Complex&view=diagram

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

Running the HFSM in a task and sending events to it:
![CleanShot 2023-04-10 at 10 55 19](https://user-images.githubusercontent.com/213467/230945519-165eda62-2e61-4e57-9571-cb2b945b62fb.png)

Running the test bench:
![CleanShot 2023-04-10 at 10 55 43](https://user-images.githubusercontent.com/213467/230945553-c6acd4cc-2de3-4413-aec0-6de506b2347f.png)
