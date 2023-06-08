# Thermistor Example

This example shows how to use the `thermistor` component to compute the
temperature from a temperature sensitive resistor (thermistor) - specifically
negative temperature coefficient (NTC) thermistors.

The first part of the example does not need any hardware and simply shows how
to use the component to compute the temperature from the resistance of the
thermistor. The second part of the example shows how to use the component with
an ESP32 to compute the temperature from the voltage divider formed by the
thermistor and a fixed resistor.

## How to use example

## Hardware Required

To run this example, you need:
* An ESP32 dev board (e.g. ESP32-WROVER Kit, ESP32-S3-BOX, QtPy ESP32, etc.)
* A thermistor (the NTC), such as [this 10k NTC](https://www.mouser.com/ProductDetail/EPCOS-TDK/B57230V2103J260?qs=sGAEpiMZZMv1TUPJeFpwbr9%252BgXDsY0jec0WWx2v3cMlaf3d8k1yxlg%3D%3D)
* A 10k resistor
* A breadboard
* Jumper wires

The example is configured so that the NTC is the top resistor in a voltage
divider, with the 10k resistor on the bottom. The voltage divider is connected
to the ADC1 channel 0 input of the ESP32.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2023-06-08 at 09 26 27](https://github.com/esp-cpp/espp/assets/213467/1f1792d3-a341-431d-a926-b623d44325d8)

