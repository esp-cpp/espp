# USB Mass Storage (MSC) Example

Exposes a FAT partition in the ESP32-S3's flash as a **USB drive** using
`espp::UsbDevice`'s MSC function, and demonstrates the ownership model that lets
the firmware and a PC share one volume safely:

- While the **application** owns the medium it reads and writes files through the
  VFS at `base_path` (`fopen`, `std::fstream`, `std::filesystem`). A connected
  host sees the drive as "no medium".
- When a **host** mounts the device it takes the medium: the application's
  `base_path` is unmounted and the PC sees the FAT volume.
- When the host **ejects** the drive (or the cable is unplugged) the medium goes
  back to the application, with the host's changes.

On boot the example updates `boots.txt` (a boot counter) and creates `README.txt`
(only if it is missing, so host edits survive reboots) on the
volume and lists its files. Plug the native USB port into a PC: the drive
appears with those files. Add or edit a file, eject the drive, and the device
logs the updated directory listing.

## Build & flash

```sh
idf.py -p <PORT> flash monitor   # console is on UART0 (USB-UART adapter)
```

The console is on **UART0**: on the ESP32-S3 USB-Serial-JTAG shares the native
USB port's PHY with USB-OTG, which the mass storage interface takes over.

The example's `sdkconfig.defaults` enables `CONFIG_TINYUSB_MSC_ENABLED`, uses a
custom `partitions.csv` with a 1 MiB `storage` FAT partition, and selects
**4096-byte wear-levelling sectors** with a matching MSC buffer:

```
CONFIG_WL_SECTOR_SIZE_4096=y
CONFIG_TINYUSB_MSC_BUFSIZE=4096   # esp_tinyusb requires >= CONFIG_WL_SECTOR_SIZE
```

## Flash write speed and sector size

NOR flash can only be erased in 4 KiB blocks, and esp_tinyusb erases a range
before writing it. How much work one host write costs depends on the
wear-levelling sector size:

| `CONFIG_WL_SECTOR_SIZE` | Host sector | Flash work per host write | Reset during an erase |
|---|---|---|---|
| **4096** (this example) | 4 KiB | 1 erase + 1 write | loses only the write in progress |
| 512, `WL_SECTOR_MODE_SAFE` | 512 B | read the 4 KiB block, back it up (erase + write), write a transaction record (erase + write), erase the block, restore the other 7 sectors, clear the record (erase), write -- **4 erases** | safe |
| 512, `WL_SECTOR_MODE_PERF` | 512 B | read the block into RAM, erase, restore, write -- 1 erase | **loses the whole 4 KiB block** |

A flash erase takes tens of milliseconds, and a host editing even a tiny file
writes many sectors: the data, the FAT (often two copies), the directory entry
and timestamps, plus host metadata such as macOS's `._name` AppleDouble files and
`.fseventsd` logs. With 512-byte Safety-mode sectors that adds up to several
seconds for a ~100-byte edit; with 4096-byte sectors it is a fraction of that.

Trade-offs of 4096-byte sectors:

- The host sees a drive with **4 KiB logical sectors**. Current macOS, Linux and
  Windows handle that; some old or embedded hosts only accept 512-byte sectors.
- FatFs keeps a sector-sized buffer per mounted volume and per open file, so each
  costs **4 KiB of RAM** instead of 512 B (and the MSC buffer is 4 KiB).
- A cluster is at least one sector, so every file, however small, occupies at
  least **4 KiB on the volume**; a small partition fits fewer files.
- Changing the sector size changes the on-flash layout: **erase the partition**
  after switching either way (below).

To reduce host metadata writes on macOS, create `.fseventsd/no_log` and
`.metadata_never_index` on the volume, and run
`defaults write com.apple.desktopservices DSDontWriteUSBStores -bool true` to stop
`.DS_Store` files.

### SD cards avoid all of this

An SD card has its own controller that does erase-block management and wear
levelling internally, so esp_tinyusb writes its 512-byte sectors straight to the
card (`sdmmc_write_sectors()`): no ESP-side wear-levelling layer, no
read-modify-erase and no sector-size choice to make. Host writes then run at the
card's speed (limited mainly by full-speed USB, about 1 MB/s), and an SD card
holds far more than a flash partition. For storage a PC writes to regularly,
prefer an SD card (see below) and keep flash media for small, rarely changed data.

### Starting over

A volume written with a different sector size, or damaged by a reset in
Performance mode (a root directory of unreadable entries, missing files), must be
erased so the example can format it again:

```sh
idf.py erase-flash flash   # or: esptool.py erase_region 0x110000 0x100000
```

## Using an SD card instead

Initialize the card (SDMMC or SDSPI host) but do **not** mount it with
`esp_vfs_fat_*_mount()` — the MSC function mounts it at `base_path` itself.
`espp::SdCard` (the `sdcard` component) keeps those two steps apart, and every
espp BSP with a microSD slot exposes its card through `sdcard()`:

```cpp
espp::SdCard::Config sd_config;
sd_config.interface = espp::SdCard::SdmmcConfig{/* pins */};
sd_config.mount_on_initialize = false; // probe only; the MSC function mounts it
espp::SdCard sdcard(sd_config);
sdcard.initialize();

espp::UsbDevice::MscMedium card;
card.type = espp::UsbDevice::MscMedium::Type::SdCard;
card.sd_card = sdcard.card();
card.base_path = "/sdcard";
msc.media = {card};            // or {card, flash} for two drives
```

With a BSP, call `initialize_sdcard(...)`, then `sdcard_component()->unmount()`
before handing `sdcard()` to the MSC function.

SD card media need a target with an SDMMC host peripheral (ESP32-S3 / -P4), even
when the card is wired to SPI.

## Example Output

First boot on an ESP32-S3 (the partition has no filesystem yet, so it is
formatted), then a macOS host mounts the drive, adds two files, and ejects it:

```console
I (342) main_task: Calling app_main()
[MSC/I][0.342]: Starting USB mass storage example
W (342) tinyusb_msc_storage: Mount failed, trying to format the drive
[MSC/I][1.142]: medium 0 now owned by the app
[UsbDevice/I][1.232]: MSC medium 0: volume label set to 'ESPP MSC'
[UsbDevice/I][1.232]: MSC medium 0: storage (1000 KiB) at '/msc', owned by the application
I (1402) TinyUSB: TinyUSB Driver installed on port 0
[UsbDevice/I][1.402]: Initialized native USB device (VID=0x1209 PID=0x0d32) cdc=false vendor=false hid=false xinput=false msc=1
[MSC/I][1.422]: volume: 2000 sectors x 512 bytes = 1000 KiB
[MSC/I][2.152]: boot #1 recorded on the volume
[MSC/I][2.152]: Files on the volume:
[MSC/I][2.152]:   boots.txt (2 bytes)
[MSC/I][2.152]:   README.txt (104 bytes)
[MSC/I][2.152]: Ready. Connect the native USB port to a PC; eject the drive to hand it back.
[MSC/I][12.732]: medium 0 now owned by the host
[MSC/I][206.992]: medium 0 now owned by the app
[MSC/I][207.462]: Files on the volume:
[MSC/I][207.462]:   boots.txt (2 bytes)
[MSC/I][207.462]:   README.txt (194 bytes)
[MSC/I][207.462]:   .fseventsd/ (0 bytes)
[MSC/I][207.462]:   ._README.txt (4096 bytes)
[MSC/I][207.472]:   .TemporaryItems/ (0 bytes)
[MSC/I][207.472]:   test_item_1.md (15 bytes)
[MSC/I][207.482]:   some_other_thing.txt (30 bytes)
```

(The dot-files are macOS metadata written by the host. The TinyUSB device
descriptor summary esp_tinyusb prints at install is omitted.)
