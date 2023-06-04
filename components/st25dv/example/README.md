# St25dv Example

This example shows how to configure a `espp::St25dv` class to communicate with
an ST25DV reprogrammable NFC tag.

![image](https://user-images.githubusercontent.com/213467/208487542-9bb15cdc-ae4f-41a9-ba5a-b71adfa3b7e7.png)

## How to use example

### Hardware Required

This example is designed to be used with an [Adafruit ST26DV16K
breakout](https://www.adafruit.com/product/4701), connected via I2C.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![output](https://user-images.githubusercontent.com/213467/209173473-28d0b3f3-c636-4a48-ab0a-4370d1d9ab9d.png)
![output 2](https://user-images.githubusercontent.com/213467/208487187-237ba823-0ded-4b71-adc5-17332fb4ad15.png)

Reading tag containing text:
![phone 1](https://user-images.githubusercontent.com/213467/208488067-03cba102-9cb0-4412-bb9e-a922bdfbefff.png)
Open web link:
![phone 2](https://user-images.githubusercontent.com/213467/208943100-b7e8a7c5-7cbc-42e4-8b1a-235e753649f8.png)
Bluetooth pairing:
![phone 3](https://user-images.githubusercontent.com/213467/209022013-cfb380f0-0bc0-42bb-898e-fe3053007a5f.png)
