# espp_ota — OTA over USB from the command line

A small, pure-Python host tool that updates an espp device over USB using the
espp `stream_frame` framing + OTA stream protocol (dispatcher **module 0**) — the
same protocol the on-device [`ota` example](../example/) serves and
[`ota_console.html`](../web/ota_console.html) drives from the browser.

It talks to the device's USB **vendor (WebUSB)** interface (`bInterfaceClass
0xFF`, one bulk IN + one bulk OUT endpoint). The frame codec and OTA protocol are
standard-library only; the USB transport uses [`pyusb`](https://pypi.org/project/pyusb/),
imported lazily.

## Seamless: build → OTA with `idf.py`

If your project uses the espp `ota` component, its `project_include.cmake`
registers an `ota-usb` build target, so you can build and flash over USB in one
step (just like `idf.py flash` does over the serial bootloader):

```sh
pip install pyusb            # once (libusb backend: `brew install libusb`, `apt install libusb-1.0-0`)
idf.py ota-usb              # builds the app, then OTAs it over USB
# or, equivalently / on CMake < 3.19:
idf.py build ota-usb
```

Override the target device without editing anything (the tool reads these):

```sh
ESPP_OTA_PID=0x1234 idf.py ota-usb
```

## Standalone CLI

Run it directly for full control (or when you already have a `.bin`):

```sh
python -m espp_ota flash build/my_app.bin        # BEGIN -> stream -> FINISH
python -m espp_ota flash build/my_app.bin --pid 0x1234 --chunk-size 2048
python -m espp_ota list                          # list matching USB devices
python -m espp_ota discover                       # probe the device's dispatcher
```

Installed with the espp wheel it's also available as the `espp-ota` command
(`pip install "espp[usb]"`).

## Library use

```python
from espp_ota import OtaClient, UsbVendorTransport

with open("build/my_app.bin", "rb") as f:
    image = f.read()

with UsbVendorTransport() as t:                    # default VID/PID 0x1209:0x0d32
    OtaClient(t, progress=lambda w, tot: print(w, "/", tot)).flash(image)
```

## Protocol

`module = 0`; requests are host→device, replies device→host (reply flag set).
Flow control is one request in flight — each request waits for its OK/ERROR
reply before the next is sent.

| type | name | dir | payload |
|------|------|-----|---------|
| 0x01 | BEGIN | host→dev | u32 image_size (0 = unknown/streaming) |
| 0x02 | DATA | host→dev | image bytes (1..4096) |
| 0x03 | FINISH | host→dev | — (validate + activate) |
| 0x04 | ABORT | host→dev | — |
| 0x05 | OK | dev→host | u32 bytes_received |
| 0x06 | ERROR | dev→host | u32 code + utf-8 message |
| 0x07 | PROGRESS | dev→host | u32 written, u32 total |

The wire framing is `espp::stream_frame` v2 (magic `0x4F54`, CRC-32); see
`espp_ota/frame.py`. Host tests (codec + a full OTA against a mock device) live
in `tests/test_ota_host.py` and run with plain `python3`.

## Output

The tool draws a [`rich`](https://pypi.org/project/rich/) progress bar (spinner,
bar, %, bytes, transfer speed, ETA) and colorizes status / error lines. Under
`idf.py ota-usb` the tool's stdout/stderr are captured pipes, so the bar is drawn
straight to the controlling terminal (`/dev/tty`, `CONOUT$` on Windows) and still
animates in place. Without a terminal (CI / redirected output) it prints periodic
plain-text lines instead. `rich` is optional — the output degrades to a plain
`\r` bar or text without it. It ships in the ESP-IDF Python environment (so
`idf.py ota-usb` already has it) and is pulled in by `pip install "espp[usb]"`.

## Requirements

- Python 3.8+
- `pyusb` + a libusb backend (only for the actual USB transport):
  - macOS: `brew install libusb`
  - Linux: `apt install libusb-1.0-0` (add a udev rule for non-root access)
  - Windows: the device advertises WebUSB + MS-OS-2.0, so WinUSB binds
    automatically; otherwise bind it once with [Zadig](https://zadig.akeo.ie/).
