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

- **The server (TV) certificate is not verified.** The TV presents a self-signed
  certificate, so the TLS client does not validate the peer certificate chain and
  the connection is not protected against an active man-in-the-middle on the local
  network. Pairing binds to the server's public key via the hashed pairing code,
  but nothing pins that key afterwards. Because no CA/bundle is supplied, esp-tls
  requires the insecure "skip server cert verify" option to be compiled in (see
  the required configuration below); without it `esp_tls_conn_new_sync()` fails
  with `ESP_ERR_MBEDTLS_SSL_SETUP_FAILED` ("No server verification option set").
- **No post-pairing certificate pinning.** After the one-time pairing exchange the
  server's public key is not stored or checked on subsequent connections, so a
  different host answering on the same address would be trusted.
- **The client private key is stored unencrypted in NVS.** The generated client
  certificate and its RSA private key are persisted in the configured NVS
  namespace in plaintext. Anyone with physical/flash read access to the device can
  recover the key and impersonate this remote. Use flash encryption if that is a
  concern.

## Required configuration

The TV's certificate is self-signed, so the esp-tls client is set up with no
CA/certificate-bundle. esp-tls only permits that when the insecure skip-verify
option is enabled, so your project's `sdkconfig.defaults` (or menuconfig) **must**
set:

```
CONFIG_ESP_TLS_INSECURE=y
CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY=y
```

Without these, both the pairing (`6467`) and control (`6466`) TLS connections
fail immediately with `ESP_ERR_MBEDTLS_SSL_SETUP_FAILED`. The example already
enables them in its `sdkconfig.defaults`.

### Interactive console (pairing code entry)

Pairing is interactive: the TV shows a 6-character code that you must type back
into the device. Reading it requires a console whose VFS driver is installed for
blocking stdin. The example calls a small `configure_console()` helper at startup
that installs the driver for the configured console, and selects the ESP32-S3
native USB-Serial-JTAG console in `sdkconfig.defaults`:

```
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
```

so the code you type into `idf.py monitor` reaches the device over the same USB
port. If your board exposes its console on a UART bridge instead, set
`CONFIG_ESP_CONSOLE_UART_DEFAULT=y`; the helper handles both. (This mirrors what
`espp::Cli::configure_stdin_stdout()` does.)

## Example

See [./example](./example) for a WiFi STA example that discovers, pairs, connects,
and sends commands. It has two UIs, selected with a CMake flag:

```
idf.py -DATV_BOARD=console build     # serial monitor (default)
idf.py -DATV_BOARD=tdeck   build     # LilyGo T-Deck: screen + keyboard
```

Board selection changes the built components and the flash/PSRAM config, which
IDF resolves before Kconfig, so it is a CMake flag rather than a Kconfig choice.
Each mode lists the exact espp component directories it needs (avoiding the whole
`components/` tree so unrelated managed dependencies are never pulled in) and
applies a matching `sdkconfig.defaults[.tdeck]`.

- **console**: reads the pairing code and prints status over the serial monitor,
  then runs a short scripted command demo. Best when you have no supported board
  or want a quick CI-friendly build.
- **tdeck**: uses the T-Deck's screen for status/prompts and its keyboard to type
  the pairing code and drive the remote live. This is far easier than running
  back to the PC to type a code before it times out. Control mapping:
  arrows / **WASD** = D-pad, **enter** = select, **h** = home, **b** = back,
  **space** = play/pause, **, .** = volume down/up, **n/p** = next/previous,
  **m** = mute, **q** = quit.

The board UI is implemented in `example/main/remote_ui.hpp` behind a small
`RemoteUi` interface, so additional boards can be added by implementing it.
