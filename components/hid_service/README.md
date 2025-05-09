# BLE HID Service Example

https://components.espressif.com/components/espp/hid_service/badge.svg

The `HidService` implements the standard BLE HID service, providing dynamic and
configurable HID input, output, and feature reports from a BLE peripheral to a
BLE central.

The `hid_service` component is designed to be used with a NimBLE host stack.

## Example

The [example](./example) shows how to use the `espp::HidService` class together
with the `espp::BleGattServer` class to create and manage a BLE GATT server that
provides an HID service. It uses the `hid-rp` component's
`espp::GamepadReport<>` to define a HID gamepad report descriptor and generate
input reports.
