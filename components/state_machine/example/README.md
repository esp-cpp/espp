# Hierarchichal Finite State Machine (HFSM) Example

This example shows an example of running the below HFSM on an ESP32 in a
real-world scenario (e.g. spawning events from one or more threads and running
the HFSM in its own thread) as well as in a test-bench scenario (e.g. running a
CLI to manually spawn events and trace the execution). For more information, see
[webgme-hfsm](https://github.com/finger563/webgme-hfsm).

![hfsm](https://user-images.githubusercontent.com/213467/230950083-d4d8a483-31a7-43ac-8822-b1e28d552984.png)

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
