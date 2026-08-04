# BQ27220 Example

This example shows how to use the `espp::Bq27220` driver to talk to a TI BQ27220
battery fuel gauge over I2C and read the battery:
* Voltage (mV)
* Current (mA)
* State of charge (%)
* Temperature (°C)

## How to use example

### Hardware Required

This example requires a connection (via I2C) to a board with a BQ27220 battery
fuel gauge and an attached battery. Configure the I2C pins (SDA/SCL) via
`menuconfig` for your board.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

The example logs the battery voltage, current, state of charge, and temperature
in a loop.
