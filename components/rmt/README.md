# RMT (Remote Control Trasceiver) Component

[![Badge](https://components.espressif.com/components/espp/rmt/badge.svg)](https://components.espressif.com/components/espp/rmt)

The `Rmt` class provides a wrapper around the ESP32 RMT peripheral. It allows
you to send infrared signals with the ESP32. See the esp-idf documentation for
more information about the RMT peripheral.
(https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html)

The `RmtEncoder` class provides a wrapper around the ESP32 rmt encoder
functionality. It allows you to encode infrared signals with the ESP32. See the
esp-idf documentation for more information about the RMT encoder
(https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html#rmt-encoder)

The main functionality of the `Rmt` / `RmtEncoder` classes beyond what is
provided by the esp-idf is to allow the use of the RMT peripheral with c++
functions (such as with bound functions, functionals, etc.). It also provides a
simpler wrapper / interface to the user.

## Example

This example shows the use of the `espp::Rmt` and `espp::RmtEncoder` classes of
the `rmt` component for driving an LED.
