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

On boot the example writes `boots.txt` (a boot counter) and `README.txt` to the
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
512-byte wear-levelling sectors (esp_tinyusb requires
`CONFIG_TINYUSB_MSC_BUFSIZE >= CONFIG_WL_SECTOR_SIZE`).

## Using an SD card instead

Initialize the card as usual (SDMMC or SDSPI host) but do **not** mount it with
`esp_vfs_fat_*_mount()` — the MSC function mounts it at `base_path` itself —
then pass the card pointer:

```cpp
espp::UsbDevice::MscMedium card;
card.type = espp::UsbDevice::MscMedium::Type::SdCard;
card.sd_card = sd_card;        // sdmmc_card_t* from sdmmc_card_init()
card.base_path = "/sdcard";
msc.media = {card};            // or {card, flash} for two drives
```

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
