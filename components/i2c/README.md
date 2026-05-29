# I2C Component

[![Badge](https://components.espressif.com/components/espp/i2c/badge.svg)](https://components.espressif.com/components/espp/i2c)

The `i2c` component provides C++ wrappers around ESP-IDF's I2C drivers.

The new master/slave bus API is available on ESP-IDF >= 5.4.0 and is required
on ESP-IDF v6.x and newer. ESPP defaults to that driver family where it is
required, while the primary `espp::I2c` class keeps the familiar address-based
helper methods (`probe_device`, `read`, `write`, `write_read`,
`read_at_register`, etc.) for backwards compatibility and also allows explicit
per-device handles via `add_device()`.

If you want direct access to the bus/device model, the component also exposes:

- `espp::I2cMasterBus`
- `espp::I2cMasterDevice`
- `espp::I2cSlaveDevice`

A helper `I2cMenu` is provided for the bus-compatible `espp::I2c` API, and the
new-device API also includes `I2cMasterMenu`, `I2cMasterDeviceMenu`, and
`I2cSlaveMenu`.

`espp::I2cSlaveDevice` now matches ESP-IDF's callback-driven slave model more
closely: `read()` returns the next complete master-write transaction buffered by
the driver callback path, `write()` stages bytes for the next master read, and
optional request / receive callbacks are dispatched in task context rather than
directly from the ISR callback.

If you need the exact transaction size, use the `read()` overload that returns
the received length by reference. The original boolean `read()` API remains for
compatibility.

On ESP-IDF v5.5's default slave driver, the underlying receive callback does
not report the exact transaction length. In that configuration ESPP still
supports buffered slave reads and receive callbacks, but trailing unread bytes
are zero-filled and the reported receive length matches the requested read size.

## Example

The [example](./example) shows both styles:

- using `espp::I2c` as a backwards-compatible bus helper
- creating an explicit device with `i2c.add_device()`

There is not yet a dedicated slave example.
