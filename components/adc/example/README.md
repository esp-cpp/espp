# ADC Example

This example shows the use of the `espp::OneshotAdc` and the `espp::ContinuousAdc` components.

It uses the following components:
* `adc`
* `logger`
* `task`

These adc components can be used to read analog values using the ESP's built-in
ADC hardware, supporting DMA and filtering as well.

## How to use example

You can build and run this example on any esp dev board, such as the QtPy ESP32
series, the TinyPICO / TinyS3 boards, the WROVER-DevKit, and the ESP32-S3-BOX.
Make sure to set the idf target when building the example.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

![CleanShot 2023-06-21 at 09 08 47](https://github.com/esp-cpp/espp/assets/213467/e6665d20-57b8-43fb-bb55-a0de1ddae7ed)

