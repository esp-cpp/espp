# Ping Component
[![Badge](https://components.espressif.com/components/espp/ping/badge.svg)](https://components.espressif.com/components/espp/ping)

A C++ wrapper around `esp_ping` providing a functional callback interface and an optional CLI menu.

- Simple config with target host, count, interval, timeout, data size, TTL
- std::function callbacks for session start, per-reply, timeout, and end
- CLI submenu for quick testing and integration alongside other menus

## Example

The [./example](./example) directory contains a simple example demonstrating the
use of the `Ping` component and how it can be used with a CLI menu.
