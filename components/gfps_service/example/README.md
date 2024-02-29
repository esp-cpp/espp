# Google Fast Pair Service (GFPS) Service Example

This example shows how to use the `espp::GfpsService` class together with the
`espp::BleGattServer` class to create and manage a BLE GATT server that provides
support for Google Fast Pair Service (GFPS) using the
[nearby/embedded](https://github.com/google/nearby) framework.

NOTE: this example uses a pre-registered device. If you want to register your
own device(s), you must do that on the [nearby/devices
dashboard](https://developers.google.com/nearby/devices).

## How to use example

### Hardware Required

This example should run on any ESP32s3 development board as it requires no
peripheral connections.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

