# PI4IOE5V Example

This example shows how to use the `Pi4ioe5v` component to communicate (via I2C)
with a PI4IOE5V I2C GPIO expander.

It is designed to run on ESP32 series devices. Update the I2C pins in the main
file as needed for your board.

## How to use example

### Hardware Required

An ESP board with I2C connected to a PI4IOE5V expander.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)


