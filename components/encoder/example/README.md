# Encoder Example

This example shows the use of the `espp::AbiEncoder` from the `encoder`
component. It showcases both the `EncoderType::ROTATIONAL` and
`EncoderType::LINEAR` uses of the `AbiEncoder`.

## How to use example

### Hardware Required

This example requires some hardware attached which provides ABI/ABZ (quadrature
encoder) input. An example is the ABI/ABZ output provided by the `Mt6701`
encoder and the `AS5600` encoder.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
