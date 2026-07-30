# Android TV Remote Component

[![Badge](https://components.espressif.com/components/espp/android_tv_remote/badge.svg)](https://components.espressif.com/components/espp/android_tv_remote)

A practical Android TV Remote v2 client component for ESP-IDF targeting Google TV
and Chromecast devices that expose the Android TV Remote Service.

## First-release scope

- mDNS discovery via `_androidtvremote2._tcp.local`
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

## Security considerations

This component prioritizes getting a working remote onto constrained hardware and
makes several deliberate trade-offs you should be aware of:

- **The server (TV) certificate is not verified.** The TLS client connects with
  `skip_common_name` and does not validate the peer certificate chain, so the
  connection is not protected against an active man-in-the-middle on the local
  network. Pairing binds to the server's public key via the hashed pairing code,
  but nothing pins that key afterwards.
- **No post-pairing certificate pinning.** After the one-time pairing exchange the
  server's public key is not stored or checked on subsequent connections, so a
  different host answering on the same address would be trusted.
- **The client private key is stored unencrypted in NVS.** The generated client
  certificate and its RSA private key are persisted in the configured NVS
  namespace in plaintext. Anyone with physical/flash read access to the device can
  recover the key and impersonate this remote. Use flash encryption if that is a
  concern.

## Example

See [./example](./example) for a WiFi STA example that discovers, pairs, connects,
and sends a few demonstration commands while still building cleanly without a real
device present.
