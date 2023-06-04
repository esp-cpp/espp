# BLDC Haptics Example

This example shows the use of the `BldcHaptics` component to drive a BLDC motor
(such as a tiny gimbal motor) as a user input / output device that provides
haptic feedback (such as might be used as a rotary encoder input).

## How to use example

### Hardware Required

This example requires a lot of hardware such as:
* Magnetic encoder chip (this example uses `Mt6701`)
* BLDC Motor Driver chip (this example was tested with the `TMC6300 BOB` dev board)
* Some mounting hardware to mount the motor, magnet, encoder, etc.

:warning:
> NOTE: you MUST make sure that you run the example with the
> `zero_electrical_offset` value set to 0 (or not provided) at least once
> otherwise the sample will not work and could potentially damage your motor.

Currently, this is designed to be run on a `TinyS3` connected to the motor
driver and encoder via breadboard with the motor powered via a benchtop power
supply at 5V.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

This example can be re-run (by modifying the code to change the selected
`DetentConfig` from one of the predefined configurations or by making your own)
to produce various behaviors. Additionally, at the end of each demo, it will
play a haptic buzz / click using the motor.

For more information, see the documentation or the original PR:
https://github.com/esp-cpp/espp/pull/60

Some examples:

### coarse values strong detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/a256b401-6e45-4284-89c7-2dec9a49daa7

### magnetic detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/ab1ace5c-f967-4cfc-b304-7736fdb35bcb

### On / Off Strong Detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/038d79b1-7cd9-4af9-b7e8-1b4daf6a363a

### Multi-rev no detents

https://github.com/esp-cpp/espp/assets/213467/2af81edb-67b8-488b-ae7a-3549be36b8cc

## Troubleshooting

Make sure to run the example once with `zero_electrical_offset` set to 0 so that
the motor will go through a calibration / zero offset routine. At the end of
this startup routine it will print the measured zero electrical offset that you
can then provide within the code, at which point it will not need to run the
calibration routine.

You must run this calibration any time you change your hardware configuration
(such as by remounting your motor, magnet, encoder chip).

## Example Breakdown

This example is relatively complex, but builds complex haptic behavior using the
following components:

* `espp::Mt6701`
* `espp::BldcDriver`
* `espp::BldcMotor`
* `espp::BldcHaptics`
* ESP-IDF's `i2c` peripheral driver

You combine the `Mt6701` and `BldcDriver` together when creating the `BldcMotor`
and then simply pass the `BldcMotor` to the `BldcHaptics` component. At that
point, you only have to interface to the `BldcHaptics` to read the input
position or reconfigure the haptics.
