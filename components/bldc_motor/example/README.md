# BldcMotor Example

This example shows the use of the `BldcMotor` component to drive a BLDC motor
(such as a tiny gimbal motor) using Field-Oriented Control (FOC) in both
open-loop and closed-loop control schemes for both position and velocity
control.

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

### Configure the project

```
idf.py menuconfig
```

* _If there is any project configuration that the user must set for this example, mention this here._

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

### Screenshots (if appropriate, e.g. schematic, board, console logs, lab pictures):
![image](https://github.com/esp-cpp/espp/assets/213467/600fe5f4-9edb-46c3-9a6c-d35242cf1597)
![image](https://github.com/esp-cpp/espp/assets/213467/5a39bec2-9490-47dd-b5ca-74e9919a123f)


### Video

https://github.com/esp-cpp/espp/assets/213467/9a48a29f-9901-44d2-a68e-b27c9220cc24

## Troubleshooting

Make sure to run the example once with `zero_electrical_offset` set to 0 so that
the motor will go through a calibration / zero offset routine. At the end of
this startup routine it will print the measured zero electrical offset that you
can then provide within the code, at which point it will not need to run the
calibration routine.

You must run this calibration any time you change your hardware configuration
(such as by remounting your motor, magnet, encoder chip).

## Example Breakdown

This example is relatively complex, but builds bldc motor control using the
following components:

* `espp::Mt6701`
* `espp::BldcDriver`
* `espp::BldcMotor`
* ESP-IDF's `i2c` peripheral driver
* `espp::Task` for updating the target
* `espp::Task` for logging state

You combine the `Mt6701` and `BldcDriver` together when creating the `BldcMotor`
and then simply use the API provided by the `BldcMotor` to set targets for
control (position or velocity) and to get the state of the motor.
