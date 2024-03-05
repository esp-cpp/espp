# Monitor Example

This example shows how to use the `monitor` component to monitor the executing
tasks.

## How to use example

### Configure the project

```
idf.py menuconfig
```

Note: the example already configures the `FREERTOS_USE_TRACE_FACILITY` and
`FREERTOS_GENERATE_RUN_TIME_STATS` sdkconfig to enable printing the task info.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2024-03-05 at 14 36 04](https://github.com/esp-cpp/espp/assets/213467/342e4d78-deee-4361-9e58-697e6dc6f73c)
