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
