# MotorGo Plink Example

This example demonstrates how to use the `espp::MotorGoPlink` component to
initialize the hardware on the MotorGo Plink board.

By default the example is intentionally safe:

- it logs the documented motor, encoder, servo, I2C, and LED pin mappings
- it starts the user and status LEDs breathing out of phase
- it leaves the motor PWM outputs disabled unless the motor sweep demo is enabled

Optional `menuconfig` flags let you also:

- initialize and poll the four encoder inputs
- run a stronger sinusoidal motor sweep across the four channels, with a zero
  crossing window and an active command range chosen to push past typical motor
  deadband

The indicator LEDs continue pulsing while the motor sweep is active because the
motor outputs use MCPWM instead of LEDC.

## How to use example

### Hardware Required

This example is designed for the MotorGo Plink board.

The default configuration does not require motors or encoders to be connected.
If you enable the motor-sweep or encoder-polling options, connect the matching
hardware first.

### Configuration

Open the example configuration menu if you want to enable the optional hardware
exercise modes:

```bash
idf.py menuconfig
```

Then open **MotorGo Plink Example Configuration** and enable any of:

- **Enable encoder polling**
- **Enable motor sweep demo**

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)
