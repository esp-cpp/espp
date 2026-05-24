# Android TV Remote Component

[![Badge](https://components.espressif.com/components/espp/android_tv_remote/badge.svg)](https://components.espressif.com/components/espp/android_tv_remote)

A practical Android TV Remote v2 client component for ESP-IDF targeting Google TV
and Chromecast devices that expose the Android TV Remote Service.

## First-release scope

- mDNS discovery via `_googlecast._tcp.local`
- pairing over TLS on port `6467`
- control over TLS on port `6466`
- persistent client certificate and private key storage in NVS
- remote keys for D-pad, enter, back, home, search, volume, mute, and media keys
- text input through the IME batch-edit path

This release intentionally focuses on the most useful embedded control surface
instead of the full protocol.

## Usage

```cpp
#include "android_tv_remote.hpp"

espp::AndroidTvRemote remote({
    .pairing = {.client_name = "ESPP Remote"},
    .log_level = espp::Logger::Verbosity::INFO,
});

std::error_code ec;
remote.pair(
    "192.168.1.50",
    []() -> std::optional<std::string> { return std::string("A1B2C3"); },
    ec);

if (!ec && remote.connect("192.168.1.50", ec)) {
  remote.home(ec);
  remote.send_text("hello world", ec);
  remote.media_play_pause(ec);
}
```

## Notes

- The pairing code must be handled correctly; devices typically show a 6-character
  hexadecimal code during pairing.
- Client credentials are generated once and persisted in NVS so that pairing can
  survive reboots.
- Some Android TV / Google TV devices do not support every command surface.

## Example

See [./example](./example) for a WiFi STA example that discovers, pairs, connects,
and sends a few demonstration commands while still building cleanly without a real
device present.
