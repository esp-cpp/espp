# INA226 Example

This example shows how to use the `Ina226` component to communicate (via I2C)
with an INA226 current/power monitor and read voltage, current, and power.

It is designed to run on ESP32 series devices. Update the I2C pins in the main
file as needed for your board.

## How to use example

### Hardware Required

An INA226 connected to the selected I2C port with a known shunt resistor value.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)


