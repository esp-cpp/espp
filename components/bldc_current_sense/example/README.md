# BLDC Current Sense Example

This example exercises the `espp::CurrentSense` FOC current sensor without any
motor hardware. It:

- statically asserts that `espp::CurrentSense` satisfies the
  `CurrentSensorConcept` required by `espp::BldcMotor`,
- builds a small software model of the analog front-end that produces phase
  currents from a commanded `(Id, Iq)` at a given electrical angle (with a fixed
  per-channel bias, and phase C unmeasured to exercise reconstruction),
- runs `driver_align()` to capture the zero-current offsets,
- commands a known q-axis current and prints the recovered `Id`/`Iq` and signed
  DC current across a range of electrical angles, confirming the offset removal +
  Clarke/Park + phase-reconstruction pipeline.

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

The recovered `Iq` should track the commanded value (with `Id ~ 0`) at every
electrical angle, demonstrating the current-sense math end to end.
