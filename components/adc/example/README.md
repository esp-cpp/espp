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
series, the TinyPICO / TinyS3 boards, the WROVER-DevKit, and the M5Stack Tab5.
Make sure to set the idf target when building the example.

### Configure

Select which ADC unit / channels the example reads via the `ADC Example
Configuration` menu:

```
idf.py menuconfig
```

The `Hardware` option has selections for specific boards which set the ADC
unit and channels to pins that are usable (exposed) on that board:

| Hardware | Unit | Channels | Pins |
|----------|------|----------|------|
| Adafruit QtPy ESP32 Pico | 1 | 4, 5 | GPIO32 (`TX`), GPIO33 (`SCL`) |
| Adafruit QtPy ESP32-S3 | 1 | 7, 8 | GPIO8 (`A3`), GPIO9 (`A2`) |
| M5Stack Tab5 (ESP32-P4) | 1 | 0, 1 | GPIO16 / GPIO17 (expansion header) |
| Custom | any | any | your choice |

With `Custom`, set the ADC unit and the two channels yourself. Note that the
continuous (DMA) portion of the example requires a DMA-capable unit; on
ESP32 / ESP32-C3 / ESP32-S3 that is only unit 1, and on ESP32-P4 ADC2
continuous conversions currently produce all-zero samples with esp-idf
(oneshot on ADC2, e.g. the Tab5's Grove pins GPIO53/54, works fine).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

![CleanShot 2023-06-21 at 09 08 47](https://github.com/esp-cpp/espp/assets/213467/e6665d20-57b8-43fb-bb55-a0de1ddae7ed)

