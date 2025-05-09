# As5600 Magnetic Encoder

[![Badge](https://components.espressif.com/components/espp/as5600/badge.svg)](https://components.espressif.com/components/espp/as5600)

The `AS5600` magnetic encoder component provides the user a convenient way to
measure

* Raw count
* Raw radians
* Raw degrees
* Accumulated count (since the component was created)
* Accumulated radians (since the component was created)
* Accumulated degrees (since the component was created)
* Speed (rotations per minute / RPM)

It does so by spawning a task which periodically reads the magnetic encoder,
updates the accumulator, and computes the velocity. The component can be
configured to optionally filter the velocity.

The periodicity / update rate of the encoder can be configured at time of
creation.

The encoder is designed to fulfill the needs of the `BldcMotor` API, to provide
closed-loop motor control.

## Example

The [example](./example) shows the use of the `As5600` component to communicate
with an AS5600 I2C magnetic encoder chip.

