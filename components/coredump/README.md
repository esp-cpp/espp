# CoreDump (Crash Report / Flash Core Dump) Component

[![Badge](https://components.espressif.com/components/espp/coredump/badge.svg)](https://components.espressif.com/components/espp/coredump)

`espp::CoreDump` is an idiomatic espp wrapper around ESP-IDF's **flash core
dump** (`espcoredump` with `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` and a
`coredump` data partition): crash detection, a ready-to-print text **crash
report** (reset reason, crashed task, PC, raw backtrace addresses and the
exact `addr2line` command line for the build target), and **raw image access**
(size / chunked reads / erase) for downloading the complete ELF core dump over
any transport. Failures are reported via `std::error_code` (no exceptions).

`espp::CoreDumpService` layers a small request/reply protocol on the espp
`stream_frame` codec (magic `"OT"` + flags + module + type + len + payload +
CRC-32) so the core dump can be inspected over **any byte stream** — the USB
vendor (WebUSB) interface, a USB CDC (Web Serial) port, a socket, a UART. It
uses dispatcher **module 4** by default (requests `0x40..0x4F`, replies
`0xC0..0xCF` with the frame reply flag set), so the service coexists with other
framed protocols (OTA on module 0, an app protocol, ...) — and with free-form
console text — on the same stream, routed by `espp::Dispatcher`. The module id
is only a routing key: `CoreDumpService::Config::module` moves an instance to
another id (used for both the requests it accepts and the replies it sends),
but the hosted console looks for 4 until told otherwise.

The matching browser tool is
[`web/coredump_console.html`](web/coredump_console.html), hosted at
[esp-cpp.github.io/espp/apps/coredump_console.html](https://esp-cpp.github.io/espp/apps/coredump_console.html):
connect over **WebUSB** (vendor interface) or **Web Serial** (CDC — the app
doubles as a serial monitor, rendering console text and protocol frames from
the same stream), view the crash summary, download the core dump as
`core.elf`, resolve backtrace addresses against your local app `.elf`
(nearest-symbol, client-side), and erase the stored dump.

The same protocol is also spoken from the terminal by the pure-Python
[`python/espp_coredump`](python/) tool, and the component's
`idf_ext.py` turns it into an `idf.py coredump-usb` action in every project
that requires `coredump`: build, download the stored dump over USB and decode
it against the freshly built ELF in one step, with options (`--gdb` for GDB on
the core file, `--summary`, `--erase`, `--out`, `--vid`/`--pid`/`--serial`), like
ESP-IDF's own `coredump-info` / `coredump-debug`. The same action is also
available through an `idf_extension` entry point of the espp wheel, and
`project_include.cmake` keeps plain `coredump-usb` / `coredump-usb-debug`
CMake targets as a fallback. All of it is host-side only: the firmware must
store core dumps to flash (`CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH` + a `coredump`
partition) and serve a `CoreDumpService` on a USB vendor interface, as the
[example](example/main/coredump_example.cpp) does. See
[`python/README.md`](python/README.md).

## Features

- **Crash detection**: `has_core_dump()` (`esp_core_dump_image_check`)
- **Summary**: `summary()` returns the raw `esp_core_dump_summary_t`
  (crashed task, PC, backtrace / stack dump, app ELF SHA-256)
- **Text report**: `format_report()` — reset reason, panic reason, crashed
  task + PC, raw backtrace addresses with a `(corrupted)` marker (Xtensa) or
  captured stack-dump size (RISC-V), and the `addr2line` decode hint using the
  right toolchain prefix for `CONFIG_IDF_TARGET`; the last reset and the
  stored dump are reported as separate events (the image persists until
  erased, so when the current reset is not a panic the dump is labeled as
  coming from an earlier crash); abnormal resets without a
  core dump (brownout / watchdog — which write no dump — with a short hint,
  or a panic whose dump is missing) are still reported; empty string = clean
  boot history (power-on / software reset / deep-sleep wake / ...)
- **Raw image access**: `image_size()`, `read_image(offset, span, ec)`
  (partition-backed chunked reads), `erase(ec)`
- **Stream service**: `espp::CoreDumpService` — transport-agnostic; construct
  with a `send` function, then register it on a dispatcher
  (`dispatcher.register_module(service)` — it carries its module id
  (`Config::module`, default 4), `handle(frame)` and discovery
  `module_info()`), or `feed(bytes)` (internal
  resynchronizing frame parser) / `handle_frame(type, payload)` (bring your own
  parser); GET_SUMMARY / GET_SIZE / READ / ERASE requests, unknown frame types
  ignored
- **Web console**: single-file, offline-capable browser app (WebUSB + Web
  Serial) with crash summary, `core.elf` download, client-side nearest-symbol
  backtrace resolution, and erase

## Configuration

The component itself needs no Kconfig options, but the **application** must
enable the flash core dump:

```
CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y
```

and the partition table needs a dedicated core dump partition, e.g.:

```
coredump, data, coredump, ,        64K,
```

## Example

The [example](./example) runs on an ESP32-S3's native USB with a composite
device: a **vendor / WebUSB** interface and a **CDC** port that carries the
system console (`tinyusb_console_init(TINYUSB_CDC_ACM_0)`). The
`CoreDumpService` is mounted on
BOTH streams, and test-crash commands (null-pointer write, `assert(false)`,
divide-by-zero, watchdog hang) can be triggered from the CDC console or the
BOOT button to exercise the full flow: crash → flash core dump → next-boot
report → browser download / decode / erase.
