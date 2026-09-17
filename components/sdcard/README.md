# SD Card Component

[![Badge](https://components.espressif.com/components/espp/sdcard/badge.svg)](https://components.espressif.com/components/espp/sdcard)

`espp::SdCard` brings up an SD / microSD card over **SDSPI** (any target) or the
**SDMMC (SDIO)** peripheral (ESP32, ESP32-S3, ESP32-P4) and mounts its FAT
volume, keeping the two as separate steps:

- `initialize()` brings up the host (an SPI device, or an SDMMC slot with an
  optional on-chip LDO powering the card) and probes the card. Afterwards
  `card()` is a valid `sdmmc_card_t` for raw sector access or for handing to
  another owner.
- `mount()` / `unmount()` register / unregister the FAT volume at
  `Config::mount_point`, any number of times, while the card stays initialized.

ESP-IDF's `esp_vfs_fat_sd*_mount()` helpers do both in one call and own the card
while it is mounted, which gets in the way when something else needs the raw
card. The main case is USB mass storage: `espp::UsbDevice`'s MSC function hands
the card to a PC, which must not happen while the firmware has the volume
mounted.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [SD Card Component](#sd-card-component)
  - [Configuration](#configuration)
  - [API](#api)
  - [Sharing the card with a USB host (MSC)](#sharing-the-card-with-a-usb-host-msc)
  - [Example](#example)
  - [Notes](#notes)

<!-- markdown-toc end -->

## Configuration

`Config::interface` selects the wiring with a `std::variant`:

```cpp
// SDMMC, 4-bit, pins through the GPIO matrix (ESP32-S3 / -P4)
espp::SdCard::SdmmcConfig sdmmc;
sdmmc.slot = 1;
sdmmc.bus_width = 4;
sdmmc.clk = GPIO_NUM_12; sdmmc.cmd = GPIO_NUM_16;
sdmmc.d0 = GPIO_NUM_14; sdmmc.d1 = GPIO_NUM_17; sdmmc.d2 = GPIO_NUM_21; sdmmc.d3 = GPIO_NUM_18;
sdmmc.frequency_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz
sdmmc.ldo_channel = -1;                     // 4 on the ESP32-P4 (LDO_VO4 powers the SD pads)

// SPI, on a bus the BSP already initialized (shared with a display)
espp::SdCard::SpiConfig spi;
spi.host = SPI2_HOST;
spi.cs = GPIO_NUM_39;
spi.initialize_bus = false; // set true (with mosi/miso/sclk) to let SdCard own the bus

espp::SdCard::Config config;
config.interface = sdmmc;            // or spi
config.mount_point = "/sdcard";
config.mount_on_initialize = true;   // false: probe only, mount() later
config.format_if_mount_failed = false; // never wipe an unknown card by default
config.max_files = 5;
config.allocation_unit_size = 16 * 1024;
```

## API

- `bool initialize(std::error_code&)` — host + card probe (+ mount by default).
  `no_such_device` means no card answered: check the card, wiring and pull-ups.
- `bool mount(ec)` / `bool unmount(ec)` — the FAT volume at `mount_point()`.
  `mount()` reports `no_such_device` when the card has no FAT filesystem and
  `format_if_mount_failed` is off.
- `bool format(ec)` — create a fresh FAT filesystem (erases the card), remounting
  afterwards if the volume was mounted.
- `sdmmc_card_t *card()` — the initialized card (stable for the object's life).
- `card_info()` — name, capacity, sector size, bus width / clock, SDHC, MMC.
- `volume_info()` — total / free bytes while mounted.
- `is_initialized()`, `is_mounted()`, `interface()`, `mount_point()`,
  `print_info()`, `deinitialize(ec)` (also done by the destructor).

## Sharing the card with a USB host (MSC)

Probe without mounting, then hand `card()` to the MSC function, which mounts the
card at its own path while the application owns it and unmounts it while the PC
does:

```cpp
espp::SdCard::Config config;
config.interface = sdmmc;
config.mount_on_initialize = false;
espp::SdCard sdcard(config);
sdcard.initialize();

espp::UsbDevice::MscMedium medium;
medium.type = espp::UsbDevice::MscMedium::Type::SdCard;
medium.sd_card = sdcard.card();
medium.base_path = "/sdcard";
```

Do not `mount()` here while the MSC function has the card: FatFs and the USB
host would both write the volume. See the `usb_device` component's `msc_example`.

## Example

The [example](./example) probes a card over SDMMC or SPI (configured in
menuconfig), records a boot counter, lists the files, and unmounts / remounts the
volume while the card stays initialized.

## Notes

- SDMMC needs `SOC_SDMMC_HOST_SUPPORTED` (ESP32, ESP32-S3, ESP32-P4). The ESP32
  routes SDMMC through fixed pins and ignores the pins in `SdmmcConfig`; the S3
  and P4 use the GPIO matrix.
- SDSPI runs the card in 1-bit SPI mode at up to 20 MHz; SDMMC 4-bit at 40 MHz is
  several times faster.
- `format_if_mount_failed` and `format()` erase the card. They use FatFs's
  `f_mkfs` on the card's own drive.
- One `SdCard` per card; the object is not copyable.
