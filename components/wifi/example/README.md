# WiFi Example

This example shows the use of the `espp::WifiSta` for connecting to existing
wifi networks, and the `espp::WifiAp` for creating a wifi access point, both of
which are provided by the `wifi` component.

## How to use example

### Configure the project

```
idf.py menuconfig
```

Since this example tests the `espp::WifiSta` which acts as a station and
connects to a pre-existing WiFi network, you should configure the wifi network
you want to connect to.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
