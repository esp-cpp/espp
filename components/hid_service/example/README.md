# BLE HID Service Example

This example shows how to use the `espp::HidService` class together with the
`espp::BleGattServer` class to create and manage a BLE GATT server that provides
an HID service. It uses the `hid-rp` component's `espp::GamepadReport<>` to
define a HID gamepad report descriptor and generate input reports.

https://github.com/esp-cpp/espp/assets/213467/36d3d04d-1d8e-4b1d-9661-4ce115c7e9cc

https://github.com/esp-cpp/espp/assets/213467/fd64e526-8c63-4456-8235-056edb418135


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

![CleanShot 2024-02-28 at 17 23 49](https://github.com/esp-cpp/espp/assets/213467/80199bb6-15e8-4396-af4a-d9a4b8b95ace)
