# SD Card Example

Brings up a microSD card with `espp::SdCard` over **SDMMC (SDIO)** or **SPI**
and shows the two-step model the component adds on top of ESP-IDF: the card is
probed once, and its FAT volume is mounted and unmounted while the card stays
initialized. It records a boot counter on the card, lists the files, unmounts,
and mounts again.

## How to use example

### Hardware Required

An ESP32-S3 (or ESP32 / ESP32-P4) with a microSD slot. The default pins are the
LilyGo T-Dongle-S3's slot (SDMMC, 4-bit). Change the interface and pins under
`idf.py menuconfig` → *SD Card Example Configuration*; an SDSPI card only needs
MOSI / MISO / SCLK / CS. On the ESP32-P4 leave the LDO channel at 4 (the chip
powers its SD pads from that LDO).

The card must carry a FAT filesystem unless you enable *Format the card if it
has no FAT filesystem* (which erases it).

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Handing the card to a USB host

The point of the split is that the card can be used by something other than the
firmware's FAT mount. With `espp::UsbDevice`'s MSC function the same card
becomes a USB drive; the device mounts it at its own path while the application
owns it, and unmounts it while the PC has it:

```cpp
espp::SdCard::Config config;
config.mount_on_initialize = false; // the MSC function mounts / unmounts it
config.interface = sdmmc;           // as above
espp::SdCard sdcard(config);
sdcard.initialize();

espp::UsbDevice::MscMedium medium;
medium.type = espp::UsbDevice::MscMedium::Type::SdCard;
medium.sd_card = sdcard.card();
medium.base_path = "/sdcard";
espp::UsbDevice::MscFunction msc;
msc.media = {medium};
```

See the `usb_device` component's `msc_example` for the rest.
