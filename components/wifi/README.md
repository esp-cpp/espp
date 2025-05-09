# WiFi Component

[![Badge](https://components.espressif.com/components/espp/wifi/badge.svg)](https://components.espressif.com/components/espp/wifi)

The `wifi` component provides classes implementing various functionality useful
to WiFi-enabled devices.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [WiFi Component](#wifi-component)
  - [WiFi Access Point (AP)](#wifi-access-point-ap)
  - [WiFi Station (STA)](#wifi-station-sta)
  - [Example](#example)

<!-- markdown-toc end -->

## WiFi Access Point (AP)

The WiFi access point enables the ESP to host its own WiFi network to which
other devices can connect.

## WiFi Station (STA)

The WiFi station enables the ESP to scan for and connect to an exising WiFi
access point.

## Example

The [example](./example) shows the use of the `espp::WifiSta` for connecting to
existing wifi networks, and the `espp::WifiAp` for creating a wifi access point,
both of which are provided by the `wifi` component.

