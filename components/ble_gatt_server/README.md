# BLE GATT Server Component

https://components.espressif.com/components/espp/ble_gatt_server/badge.svg

The `ble_gatt_server` component provides a few different classes for
implementing standard / common GATT server requirements, including the server
itself, the Device Information Service, and the Battery Service.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [BLE GATT Server Component](#ble-gatt-server-component)
  - [BLE Gatt Server](#ble-gatt-server)
  - [Battery Service](#battery-service)
  - [Device Info Service ](#device-info-service)
  - [Generic Access Service](#generic-access-service)
  - [Example](#example)

<!-- markdown-toc end -->

## BLE Gatt Server

The `BleGattServer` implements the standard BLE GATT server which has various
APIs for controlling the server and adding services. It automatically builds and
adds standard battery service and device information service.

It is designed to be used with NimBLE host stack.

## Battery Service

The `BatteryService` implements the standard BLE battery service, providing
battery state information from a BLE peripheral to a BLE central.

It is designed to be used with NimBLE host stack.

## Device Info Service 

The `DeviceInfoService` implements the standard BLE device information service,
providing device information from a BLE peripheral to a BLE central.

It is designed to be used with NimBLE host stack.

## Generic Access Service

The `GenericAccessService` implements the required standard BLE Generic Access
service, providing device information from a BLE peripheral to a BLE central.

It should be noted that as a developer, you are not required to use this
service, as one is created for you automatically by the BLE stack.

I'm not really sure why I created this file, but I have so here we are.

## Example

The [example](./example) shows how to use the `espp::BleGattServer` class to
create and manage a BLE GATT server.

