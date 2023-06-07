# Ads7138 Example

This example shows the use of the `Ads7138` component to communicate with an
ADS7138 I2C analog to digital converter using the I2C peripheral of the ESP32.

It uses the `task` component to periodically read two channels of analog data
and one channel of digital input from the ADS7138, set one channel of digital
output, and print the data in CSV format using the `format` component.

This example is designed to be run on a QtPy ESP32S3, with the BP-ADS7128
boosterpack plugged in. We connect the I2C pins on the QtPy Qwiic header (3.3V,
GPIO41 SDA and GPIO40 SCL, and GND) to J1 of the BP-ADS7128: 3.3V (pin 1), SCL
(pin 9), SDA (pin 10), and GND (pin 22). We also connect the 5V pin on the QtPy
header to the 5V pin on the BP-ADS7128 (pin 21 of J1). We then run this example,
and we should see the raw ADC values printed to the console. Finally we connect
a jumper wire between the ALERT pin of the BP-ADS7128 (pin 13 of J3) and the A0
pin of the QtPy (GPIO18). We should see the ALERT pin go low when the joystick
button is pressed. We use a GPIO interrupt handler + task to detect the falling
edge of the ALERT pin, and print a message to the console.

Table of connections between QtPy and BP-ADS7128:

| QtPy            | BP-ADS7138        |
|:----------------|:------------------|
| 3.3V (Qwiic)    | 3.3V (J1 pin 1)   |
| GND (Qwiic)     | GND (J1 pin 22)   |
| I2C SDA (Qwiic) | SDA (J1 pin 10)   |
| I2C SCL (Qwiic) | SCL (J1 pin 9)    |
| 5V              | 5V (J1 pin 21)    |
| A0 (GPIO18)     | ALERT (J3 pin 13) |


Connected to the BP-ADS7128, we have a Adafruit Thumb Joystick. The X axis
of the joystick is connected to channel 1 of the BP-ADS7128, and the Y axis
of the joystick is connected to channel 3 of the BP-ADS7128. The joystick
also has a button, which is connected to channel 5 of the BP-ADS7128. We
should see the raw ADC values of the joystick printed to the console, and
when we press the button on the joystick, we should see the digital value
change from 1 to 0.

Table of connections between BP-ADS7128 and Adafruit Thumb Joystick:

| BP-ADS7128      | Joystick      |
|:----------------|:--------------|
| CH1 (J5 pin 2)  | X axis (Xout) |
| GND (J5 pin 4)  | GND           |
| CH3 (J5 pin 6)  | Y axis (Yout) |
| CH5 (J5 pin 8)  | Button (Sel)  |
| 3.3V (J1 pin 1) | VCC           |

NOTE: The analog inputs on J5 of the BP-ADS7128 are connected via a 10k
resistor in series to the actual analog input of the ADS7128. This is to
protect the ADS7128 from overvoltage. Therefore you may want to add a pull-up
resistor to the digital input (Sel) of the joystick, so that when the button
is not pressed, the digital input is pulled high. Otherwise, the digital
input will be floating, and the ADS7128 will read a random value.

On the BP-ADS7128, we also have an LED connected to channel 7. This LED is
active low, so when we set the digital output to 1, the LED should turn off,
and when we set the digital output to 0, the LED should turn on. We can
change the digital output value by pressing the button on the joystick.

![image](https://github.com/esp-cpp/espp/assets/213467/844ceebf-81aa-4e8a-8fb4-4e4f6f7d12a8)

## How to use example

### Hardware Required

To run this example, you need:
* [QtPy ESP32S3](https://www.adafruit.com/product/5426)
* [BP-ADS7128](https://www.ti.com/tool/BP-ADS7128)
* [Adafruit Thumb Joystick](https://www.adafruit.com/product/512)
* 1 kOhm resistor (between Joystick Select and 3.3V)
* Some dupont wires
* QwiiC cable (JST-SH to Dupont)

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2023-06-07 at 11 54 43](https://github.com/esp-cpp/espp/assets/213467/49d3b569-e5cd-4575-be27-3f20e8031f3b)
![CleanShot 2023-06-06 at 13 46 56](https://github.com/esp-cpp/espp/assets/213467/cf68a4d9-4f71-4751-b00e-1ebdadcd3e88)
![CleanShot 2023-06-06 at 13 51 55](https://github.com/esp-cpp/espp/assets/213467/e1149279-00c2-42d3-a530-f6005c35e69c)

### Example videos

https://github.com/esp-cpp/espp/assets/213467/f1c5e0df-35ca-4c15-b8cb-d1455f7dbd67

https://github.com/esp-cpp/espp/assets/213467/6c56c2cf-8789-4452-beca-ffdaf68f13cb
