# BDC Driver Example

This example shows how to use the `espp::BdcDriver` component to drive a brushed
DC motor through a dual-PWM motor driver.

## How to use example

### Hardware Required

This example requires a brushed DC motor driver or H-bridge that accepts two PWM
inputs for a single motor channel.

Update the `motor_gpio_a` and `motor_gpio_b` constants in
`example/main/bdc_driver_example.cpp` to match your hardware before flashing the
example.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)
