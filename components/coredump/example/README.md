# CoreDump Example

This example shows how to use the espp `coredump` component to capture panics
to the flash core-dump partition and inspect them from a browser, on an
ESP32-S3's **native USB** port:

- On boot it prints the previous crash report (`espp::CoreDump::format_report()`):
  reset reason, panic reason, crashed task + PC, raw backtrace addresses and
  the exact `addr2line` command line — or a brownout / watchdog hint when no
  core dump exists for the reset.
- It brings up a composite USB device (`espp::UsbDevice`) with:
  - a **vendor / WebUSB** interface carrying the framed core-dump protocol, and
  - a **CDC** port carrying the **system console**
    (`tinyusb_console_init(TINYUSB_CDC_ACM_0)`)
    *and* the same framed protocol on the same stream.
- `espp::CoreDumpService` is mounted on BOTH streams, so the web console
  ([`../web/coredump_console.html`](../web/coredump_console.html), hosted at
  [esp-cpp.github.io/espp/apps/coredump_console.html](https://esp-cpp.github.io/espp/apps/coredump_console.html))
  can connect over **WebUSB** or **Web Serial**, show the crash summary,
  download the full ELF core dump (`core.elf`), resolve backtrace addresses
  against your local `build/coredump_example.elf`, and erase the stored dump.
- Test crashes can be triggered from the CDC console (type `help`) or the BOOT
  button:
  - `crash` — null-pointer write (StoreProhibited panic)
  - `assert` — failed `assert()`
  - `divzero` — integer divide by zero
  - `hang` — interrupts-off hang (INT_WDT reset, **no** core dump — shows the
    reset-reason-only report path)

## How to use example

### Hardware Required

An ESP32-S3 board with the **native USB-OTG** port wired to a USB connector
(e.g. ESP32-S3-DevKitC's "USB" connector, not the "UART" one). The TinyUSB
stack owns the S3's only USB PHY, so the system console is routed to the CDC
interface of this same connector.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash
```

(To see the console output, attach a serial terminal — or the web console's
Web Serial connect — to the CDC port that enumerates on the native USB
connector: `screen /dev/tty.usbmodem*`.)

### Typical flow

1. Flash + connect the native USB port; open the web console and connect
   (WebUSB or Web Serial).
2. Type `crash` (console / web-console serial input) or press BOOT — the
   device panics, writes the core dump to flash and reboots.
3. On reconnect the boot banner (and the web console's *Get summary*) shows
   the crash report with the raw backtrace addresses.
4. Download `core.elf` in the web console and decode it fully:

   ```
   espcoredump.py info_corefile --core core.elf --core-format elf build/coredump_example.elf
   ```

   or resolve just the backtrace addresses:

   ```
   xtensa-esp32s3-elf-addr2line -pfiaC -e build/coredump_example.elf <addrs>
   ```

   (The web console can also do a client-side nearest-symbol resolution if
   you hand it the `.elf`.)
5. Erase the dump from the web console; the next boot reports a clean history.
