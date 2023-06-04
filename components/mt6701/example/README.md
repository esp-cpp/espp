# Mt6701 Example

This example shows the use of the `Mt6701` component to communicate with a
MT6701 magnetic encoder chip.

## How to use example

It uses the `task` component to periodically read the raw count, position, and
velocity of the encoder, the `filters` component (specifically the
`espp::ButterworthFilter` class) to filter the raw values from the sensor, and
the `format` component to print the data to the console in CSV format.

### Hardware Required

This requires a MT6701 magnetic encoder dev board with an diametric magnet and
the MT6701 dev board should be connected to the ESP dev board you choose via
I2C.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

Here is some example output plotted using the [uart serial
plotter](https://github.com/appliedinnovation/uart_serial_plotter): 

![example output
plotted](https://user-images.githubusercontent.com/213467/205207680-25a4fa41-bc31-4f5e-bc69-c5e2a026d710.png)

For more information, see the original PR that included this component:
https://github.com/esp-cpp/espp/pull/5
