# MotorGo Axis Example

This example demonstrates how to use the `espp::MotorGoAxis` component to
initialize the hardware on the MotorGo Axis board.

By default the example is intentionally safe:

- it logs the documented motor, encoder, I2C, and LED pin mappings
- it starts the user and status LEDs breathing out of phase
- it initializes the two BLDC driver channels and then leaves them disabled

Optional `menuconfig` flags let you also initialize and poll the two encoder
inputs.

## How to use example

### Hardware Required

This example is designed for the MotorGo Axis board.

The default configuration does not require motors or encoders to be connected.
If you enable encoder polling, connect the matching hardware first.

### Configuration

Open the example configuration menu if you want to enable encoder polling:

```bash
idf.py menuconfig
```

Then open **MotorGo Axis Example Configuration** and enable:

- **Enable encoder polling**

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)
