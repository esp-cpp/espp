# QEMU

[![Badge](https://components.espressif.com/components/espp/qemu/badge.svg)](https://components.espressif.com/components/espp/qemu)

Board Support Package (BSP) for [Espressif's QEMU](https://github.com/espressif/qemu):
run espp applications with no hardware at all, on a laptop, in CI or in a
container. QEMU is not a board, but it emulates a few peripherals no real board
has, and this component wraps them with the same shape as the real-board BSPs:

- **Display**: the virtual RGB panel, as an LVGL display
  (`initialize_display()`). What LVGL draws appears in the QEMU window, or
  off screen in a headless run, and can be captured as a screenshot.
- **Ethernet**: the OpenCores Ethernet MAC (`initialize_ethernet()`), a real
  lwIP interface on QEMU's user-mode NAT or on a tap device.
- **SD card** (ESP32 machine only): the SD/MMC host (`initialize_sdcard()`),
  a FAT volume on an image file.

Everything else (touch, I2C, I2S, Wi-Fi, BLE, USB) is not emulated. The
component builds for the three targets the emulator supports: `esp32`,
`esp32c3` and `esp32s3`.

## Running

Install the emulators once (they are also in the `espressif/idf` Docker image):

```bash
python $IDF_PATH/tools/idf_tools.py install qemu-xtensa qemu-riscv32
```

Then, from the [example](./example):

```bash
idf.py set-target esp32 build
dd if=/dev/zero bs=1M count=64 of=sd.img          # a blank card, formatted on first use
idf.py qemu --graphics --qemu-extra-args "-drive file=sd.img,if=sd,format=raw" monitor
```

`idf.py qemu` builds the flash and eFuse images, adds the Ethernet NIC
(`-nic user,model=open_eth`) and runs QEMU; `--graphics` opens the panel
window, `monitor` attaches the serial console. See the
[QEMU guide](https://esp-cpp.github.io/espp/qemu.html) in the espp docs for
headless runs, screenshots, tap networking, GDB and the other targets.

**The display needs a display backend.** The panel driver waits for QEMU to
take each drawn area, and with `-nographic` / `-display none` nothing ever
does: the first flush spins and the application stalls there (seen on both the
ESP32 and ESP32-C3 machines). For a run without a window, keep `-display sdl`
and point SDL at its off-screen driver:

```bash
SDL_VIDEODRIVER=dummy idf.py qemu --graphics monitor
```

Screenshots: `screendump shot.ppm` in the QEMU monitor. Without the trailing
`monitor` action, `idf.py qemu` multiplexes the serial port and the QEMU
monitor on the terminal (`Ctrl-A C` switches); with it, or headless, pass
`-monitor unix:mon.sock,server,nowait` (path under 104 bytes) and write the
command to the socket from a second terminal while QEMU runs.

## Networking

With the default user-mode NIC the DHCP client gets `10.0.2.15`, the host is
`10.0.2.2` and DNS is `10.0.2.3`; outbound TCP/UDP works, inbound needs
`hostfwd` rules, and **multicast does not cross to the host**. Anything that
discovers peers by multicast (RTPS, mDNS) needs a tap device instead
(`-nic tap,...,model=open_eth`); the guide shows the Linux setup. The emulated
MAC has no multicast address filter, so ESP-IDF logs
`add mac address to filter not supported` when lwIP joins a group; the frames
are received regardless.

## API

```cpp
auto &qemu = espp::Qemu::get();
qemu.initialize_display({.width = 320, .height = 240});   // LVGL on the virtual panel
qemu.initialize_ethernet({.mode = espp::Qemu::DhcpMode::CLIENT,
                          .on_got_ip = [](esp_ip4_addr_t ip) { /* ... */ }});
qemu.initialize_sdcard({.mount_point = "/sdcard"});       // ESP32 only
```

`initialize_display()` fails on real hardware (the panel's registers do not
exist there), which doubles as an "are we in QEMU" check.
`initialize_ethernet()` needs `CONFIG_ETH_USE_OPENETH` (set in the example's
`sdkconfig.defaults`, together with `CONFIG_ETH_USE_ESP32_EMAC=n`: the chip's
own EMAC is not emulated and its driver is not needed).

## Example

The [example](./example) shows the three peripherals together: an LVGL screen
with a sweeping arc and status lines, a DHCP client on the user-mode network,
and a file written to and read back from the SD image.
