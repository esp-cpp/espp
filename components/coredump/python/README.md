# espp_coredump — core dumps over USB from the command line

A small, pure-Python host tool that pulls the stored core dump off an espp
device over USB and decodes it, using the espp `stream_frame` framing + core-dump
stream protocol (dispatcher **module 4**) — the same protocol the on-device
[`coredump` example](../example/) serves and
[`coredump_console.html`](../web/coredump_console.html) drives from the browser.

It talks to the device's USB **vendor (WebUSB)** interface (`bInterfaceClass
0xFF`, one bulk IN + one bulk OUT endpoint). The frame codec and protocol are
standard-library only; the USB transport uses [`pyusb`](https://pypi.org/project/pyusb/),
imported lazily, and the decode step hands the core file to ESP-IDF's
[`esp-coredump`](https://pypi.org/project/esp-coredump/) (which the IDF Python
environment already ships).

## Seamless: crash → decode with `idf.py`

If your project uses the espp `coredump` component, its `project_include.cmake`
registers a `coredump-usb` build target — ESP-IDF only includes a component's
`project_include.cmake` when that component is in the build, so requiring
`coredump` is all it takes. It builds the app (so the ELF matches what is on
the device), downloads the stored core dump over USB and decodes it against
that ELF, the way `idf.py coredump-info` does over the serial bootloader:

```sh
pip install pyusb esp-coredump   # once (libusb backend: `brew install libusb`, `apt install libusb-1.0-0`)
idf.py coredump-usb              # builds, then downloads + decodes the core dump
# or, equivalently / on CMake < 3.19:
idf.py build coredump-usb
```

Override the target device without editing anything (the tool reads these):

```sh
ESPP_COREDUMP_PID=0x1234 idf.py coredump-usb
```

## Standalone CLI

Run it directly for full control:

```sh
python -m espp_coredump summary                    # the crash report the device printed at boot
python -m espp_coredump size                       # stored image size (0 = no core dump)
python -m espp_coredump download                   # save the ELF core file as core.elf
python -m espp_coredump download --out crash.elf   # ... elsewhere
python -m espp_coredump download --raw             # keep the flash image (header + ELF + checksum)
python -m espp_coredump debug build/my_app.elf     # download + `esp-coredump info_corefile`
python -m espp_coredump debug build/my_app.elf --gdb   # download + `esp-coredump dbg_corefile`
python -m espp_coredump erase                      # erase the stored core dump (asks first; -y skips)
python -m espp_coredump list                       # list matching USB devices
python -m espp_coredump discover                   # list the device's dispatcher modules
```

Installed with the espp wheel it's also available as the `espp-coredump` command
(`pip install "espp[usb]"`, or `"espp[usb-ui]"` to also get the `rich` UI).

The VID/PID default to the coredump example's ids (`0x1209:0x0d36`); pass
`--vid` / `--pid` (`--pid -1` matches any product id) or set
`ESPP_COREDUMP_VID` / `ESPP_COREDUMP_PID` / `ESPP_COREDUMP_SERIAL`.

### What `debug` does

1. `GET_SIZE`, then `READ` the image in 2 KiB chunks (each `DATA` reply must echo
   the requested offset and length; a reply timeout is retried twice). Every
   request carries a correlation id that the device echoes, so a late reply of a
   timed-out request is recognised and dropped rather than taken for the retry's
   (or the next request's). Retries start only once the device's first reply
   has shown that it echoes ids; a device whose firmware predates that echo
   still works, but is never retried, since its late replies could not be told
   apart.
2. The stored image is the raw partition contents, `[flash header][ELF core
   file][checksum]`; the ELF magic is located in the first KiB and the file
   saved from there as `core.elf` (`--out` to choose). A device built with the
   binary core-dump format has no ELF magic; the raw image is saved instead and
   decoded with `--core-format raw`.
3. `esp-coredump info_corefile --core core.elf --core-format elf <app.elf>` is
   run in the foreground (`--gdb` runs `dbg_corefile`, which opens GDB on the
   core). The `esp-coredump` console script is used if installed, else
   `$IDF_PATH/components/espcoredump/espcoredump.py`; if neither is found the
   exact command to run is printed.

## Library use

```python
from espp_coredump import CoreDumpClient, UsbVendorTransport, extract_elf

with UsbVendorTransport() as t:                    # default VID/PID 0x1209:0x0d36
    client = CoreDumpClient(t, progress=lambda done, total: print(done, "/", total))
    print(client.summary())
    image = client.read_image()                    # b"" when there is no core dump

elf = extract_elf(image)                           # None when the image holds no ELF
if elf:
    open("core.elf", "wb").write(elf)
```

## Protocol

`module = 4`; requests are host→device, replies device→host (reply flag set —
the service derives it from the high bit of the type). Flow control is one
request in flight.

| type | name | dir | payload |
|------|------|-----|---------|
| 0x40 | GET_SUMMARY | host→dev | — |
| 0x41 | GET_SIZE | host→dev | — |
| 0x42 | READ | host→dev | u32 offset, u16 length (≤ 4092) |
| 0x43 | ERASE | host→dev | — |
| 0xC0 | SUMMARY | dev→host | utf-8 crash report (empty = clean boot history) |
| 0xC1 | SIZE | dev→host | u32 image size (0 = no core dump) |
| 0xC2 | DATA | dev→host | u32 offset + image bytes |
| 0xC3 | OK | dev→host | u32 (0 for ERASE) |
| 0xC4 | ERROR | dev→host | u32 code (informational) + utf-8 message |

The wire framing is `espp::stream_frame` v2 (magic `0x4F54`, CRC-32); see
`espp_coredump/frame.py`. Host tests (codec, a chunked download against a mock
device with offset verification, error mapping, ELF extraction, discovery)
live in `tests/test_coredump_host.py` and run with plain `python3`.

## Output

Like `espp_ota`, the tool draws a [`rich`](https://pypi.org/project/rich/)
progress bar for the download and colorizes status / error lines; under
`idf.py coredump-usb` the bar is drawn straight to the controlling terminal
even though idf.py captures the tool's output. `rich` is optional (plain text
without it) and ships in the ESP-IDF Python environment.

## Requirements

- Python 3.8+
- `pyusb` + a libusb backend (only for the actual USB transport):
  - macOS: `brew install libusb`
  - Linux: `apt install libusb-1.0-0` (add a udev rule for non-root access)
  - Windows: the device advertises WebUSB + MS-OS-2.0, so WinUSB binds
    automatically; otherwise bind it once with [Zadig](https://zadig.akeo.ie/).
- `esp-coredump` (or an `IDF_PATH` with `components/espcoredump/`) for the
  decode step; `download` / `summary` / `erase` work without it.
