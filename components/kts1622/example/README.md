# KTS1622 Example

This example shows how to use the `Kts1622` component to communicate (via I2C)
with an KTS1622 I2C digital IO expander.

It is currently designed to run on an ESP32-S3, but (by changing the I2C pin
definitions in the main file) can be reconfigured to run on any of the ESP32
chips.

## How to use example

### Hardware Required

This example requires a connection (via I2C) to a dev board which has an KTS1622.
To that chip should be attached some digital inputs (such as buttons).

It has been tested on a `QtPy ESP32S3` connected to a `KTS1622EUAA-MMEV01`
kts1622 evaluation kit via a qwiic / stemma qt cable.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

![CleanShot 2024-02-14 at 17 07 57](https://github.com/esp-cpp/espp/assets/213467/4a1c15fb-bcf7-43cd-bc75-3e9f22624301)
