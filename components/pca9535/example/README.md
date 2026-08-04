# PCA9535 Example

This example shows how to use the `espp::Pca9535` driver to talk to a PCA9535 /
PCA9555 16-bit I2C GPIO expander:
* configure pin directions per port
* read the input pins
* drive the output pins

## How to use example

### Hardware Required

This example requires a connection (via I2C) to a board with a PCA9535 (or the
register-identical PCA9555) I/O expander. Configure the I2C pins (SDA/SCL) via
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

The example configures the expander's ports, then logs the input pin state and
toggles an output in a loop.
