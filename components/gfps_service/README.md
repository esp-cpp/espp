# Google Fast Pair Service (GFPS) Service BLE Service Component

The `GfpsService` implements the Google Fast Pair Service, allowing first-party
fast-pair experience on Android devices using the google nearby framework. This
also supports app matching / launching, as well as Google's `Find My Device`
service. Finally, GFPS supports sharing device pairing info across that
account's google devices and personalizing the name of the device.

The `gfps_service` component is designed to be used with a NimBLE host stack.

## Example

The [example](./example) shows how to use the `espp::GfpsService` class together
with the `espp::BleGattServer` class to create and manage a BLE GATT server that
provides support for Google Fast Pair Service (GFPS) using the
[nearby/embedded](https://github.com/google/nearby) framework.
