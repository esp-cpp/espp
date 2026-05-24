# Android TV Remote Example

This example connects to WiFi in station mode, optionally discovers Google TV /
Chromecast devices with mDNS, optionally performs pairing, then tries a control
session that sends a few representative key, text, and media commands.

## Demonstrated features

- WiFi STA startup
- mDNS discovery using `_googlecast._tcp.local`
- optional pairing with a configured 6-character pairing code
- TLS control connection setup
- key injection, IME text, and media helpers

## How to use example

Configure the example through menuconfig, then build and flash:

```sh
idf.py flash monitor
```

The example compiles without a device present. At runtime it:

- skips discovery if WiFi is unavailable
- skips pairing if no target host or pairing code is configured
- logs and exits cleanly if the target does not support discovery, pairing, or control
