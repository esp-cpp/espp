# OTA Example

This example shows how to use the `espp::Ota` component to update a device's
firmware over **three transports feeding the exact same engine**:

1. **USB vendor / WebUSB** — an `espp::UsbDevice` vendor interface
   (bInterfaceClass 0xFF, WebUSB + MS OS 2.0 descriptors) carrying the framed
   OTA stream protocol from `detail/ota_stream_protocol.hpp`. Update straight
   from a Chromium browser with the hosted
   [espp OTA Console](https://esp-cpp.github.io/espp/apps/ota_console.html)
   (source: [`components/ota/web/ota_console.html`](../web/ota_console.html)) —
   no driver, no network.
2. **WiFi HTTP push** — an `esp_http_server` with `POST /ota` that streams the
   raw request body into the engine (Content-Length is the image size):

   ```bash
   curl --data-binary @build/ota_example.bin http://<ip>/ota
   # with EXAMPLE_OTA_HTTP_TOKEN configured:
   curl -H "Authorization: Bearer <token>" --data-binary @build/ota_example.bin http://<ip>/ota
   ```

   > **Security**: by default (empty `EXAMPLE_OTA_HTTP_TOKEN`) this endpoint is
   > **unauthenticated** — any peer that can reach the device can install a
   > structurally-valid image; a warning is logged at startup. Set
   > `EXAMPLE_OTA_HTTP_TOKEN` in menuconfig to require a bearer token (the
   > upload page has a matching field). A shared token is demo-grade gating
   > only: for real deployments enable **secure boot / signed images** so the
   > bootloader rejects unauthorized firmware regardless of transport.

3. **Browser HTTP upload** — `GET /ota` serves a tiny self-contained upload
   page (file picker + progress bar), so any browser on the LAN can update the
   board.

The HTTP server binds to every network interface, so the **same code works
unchanged over the espp `ethernet` component** (or any other `esp_netif`) —
Ethernet needs no separate OTA code path, just bring up its netif instead of
(or in addition to) `espp::WifiSta`.

## How to use example

### Hardware Required

An ESP32-S3 (the native USB-OTG peripheral is required for the USB / WebUSB
transport; the target is pinned in `sdkconfig.defaults`).

**Console routing.** On the ESP32-S3 the USB-Serial-JTAG console and the USB-OTG
controller share the same internal USB PHY / port, so the console cannot stay on
USB-Serial-JTAG once this example hands that port to TinyUSB — doing so
reboot-loops the device. Instead the console is set up so a **single native USB
cable carries both the OTA stream and the logs**:

- **primary console: UART0** — always available (connect a UART / USB-UART
  adapter for `idf.py monitor`); TinyUSB never touches it, so it is the safe
  fallback.
- **secondary console: USB-Serial-JTAG** — carries the early-boot / bootloader
  logs on the native USB port *before* the app brings up TinyUSB.
- once TinyUSB is up the example adds a **USB-CDC** interface and **reroutes the
  console (stdout) to it**, teeing to UART0 as well. So on the native USB port
  you see boot logs over USB-Serial-JTAG and then, seamlessly, the running app's
  logs over USB-CDC — alongside the OTA vendor / WebUSB interface on the same
  cable. (The CDC console only emits while a host has the CDC port open, and
  never blocks the app if nothing is reading it.)

### Configure

```bash
idf.py menuconfig
```

Set the WiFi SSID / password under `OTA Example Configuration` (only needed for
the HTTP transports; the USB transport works without any network).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```bash
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

The first flash writes the app into the `ota_0` slot of the factory-less
partition table (`partitions.csv`: `otadata` + `ota_0` + `ota_1`); every OTA
update alternates to the other slot.

### Update over USB (WebUSB)

1. Open <https://esp-cpp.github.io/espp/apps/ota_console.html> in a Chromium
   browser (or open `components/ota/web/ota_console.html` from disk).
2. Connect to the "espp OTA" device, pick the new `build/ota_example.bin`, and
   press Upload. The console streams BEGIN / DATA / FINISH frames (4 KiB max
   payload each, one in flight) with a progress bar; the device validates the
   image, replies OK, and restarts into it.

### Update over HTTP (WiFi or Ethernet)

- Browser: open `http://<ip>/ota`, pick the `.bin`, upload.
- CLI: `curl --data-binary @build/ota_example.bin http://<ip>/ota` — returns
  `{"status":"ok",...}` on success or a 4xx/5xx JSON error.

### Rollback semantics

`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` is set, so a freshly-installed image
boots in the `PENDING_VERIFY` state and rolls back to the previous slot on the
next reset unless it is confirmed. This example demonstrates **host-driven**
confirmation: the app deliberately does **not** mark itself valid. On boot it
logs the running partition + version, and if the image is pending verify it just
logs that it is **waiting for the host to confirm it** — a broken build could
otherwise self-validate right before failing. The **host** confirms the image
(`MARK_VALID` over the OTA protocol) once it has checked the device is healthy:
the [ota-console web app](../web/ota_console.html) prompts to confirm on
reconnect, and `espp-ota flash` auto-verifies (reconnects after the reboot,
reads status, and marks the image valid if it booted and responded). `status`,
`mark-valid`, and `rollback` (mark invalid + reboot to the previous image) are
also available to do it manually.

(If your own product prefers device self-validation instead, run your health
checks at boot and call `espp::Ota::mark_app_valid()` /
`mark_app_invalid_and_rollback()` directly — see the commented note in
`ota_example.cpp`.)

Note: `espp::Ota::finish()` only validates the image and sets the boot
partition; the restart is a separate explicit `restart()` call (this example
restarts ~750 ms after replying to the host, on any transport).

## Example Output

```
I (608) OtaExample: Running 'ota_example' version 'v1.2.3-14-g35e120b' (built Aug 19 2026 12:34:56) from partition 'ota_0' (1966080 bytes)
I (618) OtaExample: Next update will target partition 'ota_1' (1966080 bytes)
I (668) espp_UsbDevice: USB device initialized (vendor/WebUSB interface ready)
I (670) OtaExample: Routing console to USB-CDC (single cable: OTA + logs; UART0 stays teed).
I (672) OtaExample: Console is now also on USB-CDC.
I (5178) OtaExample: got IP: 192.168.1.23
I (5178) OtaExample:   browser upload page: http://192.168.1.23/ota
I (5178) OtaExample:   curl --data-binary @build/ota_example.bin http://192.168.1.23/ota
I (5188) OtaExample: HTTP OTA server ready: GET /ota (upload page), POST /ota (raw image)
I (5198) OtaExample: OTA example ready; transports: USB vendor/WebUSB, HTTP POST /ota (WiFi/Ethernet)
I (42198) Ota: incoming firmware: project 'ota_example', version 'v1.2.4', built Aug 20 2026 09:00:00 (IDF v6.0)
I (55123) Ota: finish: 1204224 bytes validated; boot partition set to 'ota_1' — call restart() to boot the new image
...reboot...
W (612) OtaExample: This image is PENDING VERIFY (first boot after an OTA update). Waiting for the host to confirm it (MARK_VALID); it rolls back on the next reset if not.
...host reconnects and confirms (ota-console prompt / `espp-ota flash` auto-verify)...
I (9051) Ota: running app marked valid; rollback cancelled
```
