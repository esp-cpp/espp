#include "sdcard.hpp"

#include <cstring>

#include <esp_idf_version.h>
#include <esp_vfs_fat.h>
#include <sd_protocol_defs.h> // SD_OCR_SDHC_CAP
#include <sdmmc_cmd.h>

// FatFs drive plumbing: the same calls ESP-IDF's esp_vfs_fat_sd*_mount() makes
// internally, used here so the card can be probed once and mounted / unmounted
// any number of times (IDF's helpers only offer both steps together before v6.1).
#include <diskio_impl.h>
#include <diskio_sdmmc.h>
#include <ff.h>

// The on-chip LDO power control (ESP32-P4 SD pads) is only compiled by ESP-IDF on
// targets with general-purpose LDOs; the header exists everywhere.
#if defined(SOC_GP_LDO_SUPPORTED) && SOC_GP_LDO_SUPPORTED &&                                       \
    __has_include(<sd_pwr_ctrl_by_on_chip_ldo.h>)
#include <sd_pwr_ctrl_by_on_chip_ldo.h>
#define ESPP_SDCARD_HAS_LDO_PWR_CTRL 1
#else
#define ESPP_SDCARD_HAS_LDO_PWR_CTRL 0
#endif

namespace espp {

namespace {
constexpr uint8_t kNoDrive = 0xFF;

std::string fat_drive_string(uint8_t pdrv) { return std::to_string(pdrv) + ":"; }

// Release the host the way IDF's helpers do: a host that takes its handle in
// deinit_p() (SDSPI device, and SDMMC slots on newer IDF) gets it.
void call_host_deinit(const sdmmc_host_t &host) {
  if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
    if (host.deinit_p)
      host.deinit_p(host.slot);
  } else if (host.deinit) {
    host.deinit();
  }
}
} // namespace

SdCard::SdCard(const Config &config)
    : BaseComponent("SdCard", config.log_level)
    , config_(config) {}

SdCard::~SdCard() {
  std::error_code ec;
  deinitialize(ec);
}

bool SdCard::initialize() {
  std::error_code ec;
  return initialize(ec);
}

bool SdCard::initialize(std::error_code &ec) {
  ec.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    logger_.warn("Already initialized");
    return true;
  }
  if (config_.mount_point.size() < 2 || config_.mount_point.front() != '/') {
    logger_.error("mount_point '{}' must be an absolute VFS path like '/sdcard'",
                  config_.mount_point);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!init_host(ec))
    return false;

  // Probe the card. sdmmc_card_init() runs the SD / MMC identification sequence
  // and fills card_ (CID / CSD / OCR, bus width, real frequency).
  logger_.info("Probing the card");
  std::memset(&card_, 0, sizeof(card_));
  const esp_err_t err = sdmmc_card_init(&host_, &card_);
  if (err != ESP_OK) {
    logger_.error("No card answered ({}): check that a card is inserted, the wiring, and the "
                  "pull-ups on CMD / D0-D3",
                  esp_err_to_name(err));
    deinit_host();
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  initialized_ = true;
  logger_.info("Card '{}' ready: {} MiB, {} kHz, {}-bit", card_.cid.name,
               static_cast<uint64_t>(card_.csd.capacity) * card_.csd.sector_size / (1024 * 1024),
               card_.real_freq_khz, 1u << card_.log_bus_width);

  if (config_.mount_on_initialize && !mount_locked(ec)) {
    // leave nothing half-done: the caller sees a clean "not initialized"
    initialized_ = false;
    deinit_host();
    return false;
  }
  return true;
}

bool SdCard::init_host(std::error_code &ec) {
  if (const auto *spi = std::get_if<SpiConfig>(&config_.interface)) {
    if (spi->cs == GPIO_NUM_NC) {
      logger_.error("SpiConfig::cs is required");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    if (spi->initialize_bus) {
      if (spi->mosi == GPIO_NUM_NC || spi->miso == GPIO_NUM_NC || spi->sclk == GPIO_NUM_NC) {
        logger_.error("SpiConfig::initialize_bus needs mosi, miso and sclk");
        ec = std::make_error_code(std::errc::invalid_argument);
        return false;
      }
      spi_bus_config_t bus{};
      bus.mosi_io_num = spi->mosi;
      bus.miso_io_num = spi->miso;
      bus.sclk_io_num = spi->sclk;
      bus.quadwp_io_num = GPIO_NUM_NC;
      bus.quadhd_io_num = GPIO_NUM_NC;
      bus.max_transfer_sz = spi->max_transfer_size;
      const esp_err_t err = spi_bus_initialize(spi->host, &bus, SDSPI_DEFAULT_DMA);
      if (err != ESP_OK) {
        logger_.error("spi_bus_initialize failed: {}", esp_err_to_name(err));
        ec = std::make_error_code(std::errc::io_error);
        return false;
      }
      bus_initialized_ = true;
    }
    host_ = SDSPI_HOST_DEFAULT();
    host_.slot = spi->host;
    host_.max_freq_khz = spi->frequency_khz;
    sdspi_device_config_t device = SDSPI_DEVICE_CONFIG_DEFAULT();
    device.host_id = spi->host;
    device.gpio_cs = spi->cs;
    device.gpio_cd = spi->card_detect;
    device.gpio_wp = spi->write_protect;
    // host_.init() is sdspi_host_init(); the device handle it returns REPLACES
    // the slot in the host struct (that is how the SDSPI host addresses the card).
    esp_err_t err = host_.init ? host_.init() : ESP_OK;
    sdspi_dev_handle_t handle = -1;
    if (err == ESP_OK)
      err = sdspi_host_init_device(&device, &handle);
    if (err != ESP_OK) {
      logger_.error("Could not attach the card to SPI host {} (cs {}): {}",
                    static_cast<int>(spi->host), static_cast<int>(spi->cs), esp_err_to_name(err));
      if (bus_initialized_) {
        spi_bus_free(spi->host);
        bus_initialized_ = false;
      }
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    host_.slot = handle;
    logger_.debug("SDSPI device on host {} cs {} at {} kHz", static_cast<int>(spi->host),
                  static_cast<int>(spi->cs), spi->frequency_khz);
    return true;
  }

  const auto &sdmmc = std::get<SdmmcConfig>(config_.interface);
#if !SOC_SDMMC_HOST_SUPPORTED
  (void)sdmmc;
  logger_.error("This target has no SDMMC host; use SpiConfig");
  ec = std::make_error_code(std::errc::function_not_supported);
  return false;
#else
  if (sdmmc.bus_width != 1 && sdmmc.bus_width != 4) {
    logger_.error("SdmmcConfig::bus_width must be 1 or 4, got {}", sdmmc.bus_width);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  host_ = SDMMC_HOST_DEFAULT();
  host_.slot = sdmmc.slot;
  host_.max_freq_khz = sdmmc.frequency_khz;
  if (sdmmc.ldo_channel >= 0) {
#if ESPP_SDCARD_HAS_LDO_PWR_CTRL
    sd_pwr_ctrl_ldo_config_t ldo{};
    ldo.ldo_chan_id = sdmmc.ldo_channel;
    sd_pwr_ctrl_handle_t handle = nullptr;
    const esp_err_t err = sd_pwr_ctrl_new_on_chip_ldo(&ldo, &handle);
    if (err != ESP_OK) {
      logger_.error("Could not power the card from LDO channel {}: {}", sdmmc.ldo_channel,
                    esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    ldo_handle_ = handle;
    host_.pwr_ctrl_handle = handle;
#else
    logger_.error("SdmmcConfig::ldo_channel: this target has no on-chip LDO for the SD pads");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  }
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = sdmmc.bus_width;
  slot.gpio_cd = sdmmc.card_detect;
  slot.gpio_wp = sdmmc.write_protect;
#if SOC_SDMMC_USE_GPIO_MATRIX
  if (sdmmc.clk != GPIO_NUM_NC)
    slot.clk = sdmmc.clk;
  if (sdmmc.cmd != GPIO_NUM_NC)
    slot.cmd = sdmmc.cmd;
  if (sdmmc.d0 != GPIO_NUM_NC)
    slot.d0 = sdmmc.d0;
  if (sdmmc.d1 != GPIO_NUM_NC)
    slot.d1 = sdmmc.d1;
  if (sdmmc.d2 != GPIO_NUM_NC)
    slot.d2 = sdmmc.d2;
  if (sdmmc.d3 != GPIO_NUM_NC)
    slot.d3 = sdmmc.d3;
#else
  if (sdmmc.clk != GPIO_NUM_NC || sdmmc.cmd != GPIO_NUM_NC || sdmmc.d0 != GPIO_NUM_NC)
    logger_.warn("This target routes SDMMC through fixed pins; the configured pins are ignored");
#endif
  esp_err_t err = host_.init ? host_.init() : ESP_OK; // sdmmc_host_init()
  const bool host_inited = err == ESP_OK;
  if (host_inited)
    err = sdmmc_host_init_slot(sdmmc.slot, &slot);
  if (err != ESP_OK) {
    logger_.error("Could not initialize SDMMC slot {}: {}", sdmmc.slot, esp_err_to_name(err));
    if (host_inited)
      call_host_deinit(host_); // the slot init failed: release the host we just brought up
#if ESPP_SDCARD_HAS_LDO_PWR_CTRL
    if (ldo_handle_) {
      sd_pwr_ctrl_del_on_chip_ldo(static_cast<sd_pwr_ctrl_handle_t>(ldo_handle_));
      ldo_handle_ = nullptr;
    }
#endif
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  logger_.debug("SDMMC slot {} {}-bit at {} kHz", sdmmc.slot, sdmmc.bus_width, sdmmc.frequency_khz);
  return true;
#endif // SOC_SDMMC_HOST_SUPPORTED
}

void SdCard::deinit_host() {
  call_host_deinit(host_);
#if ESPP_SDCARD_HAS_LDO_PWR_CTRL
  if (ldo_handle_) {
    sd_pwr_ctrl_del_on_chip_ldo(static_cast<sd_pwr_ctrl_handle_t>(ldo_handle_));
    ldo_handle_ = nullptr;
  }
#endif
  if (bus_initialized_) {
    spi_bus_free(std::get<SpiConfig>(config_.interface).host);
    bus_initialized_ = false;
  }
  host_ = sdmmc_host_t{};
}

bool SdCard::mount() {
  std::error_code ec;
  return mount(ec);
}

bool SdCard::mount(std::error_code &ec) {
  ec.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  return mount_locked(ec);
}

bool SdCard::mount_locked(std::error_code &ec) {
  if (mounted_)
    return true;
  uint8_t pdrv = kNoDrive;
  if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == kNoDrive) {
    logger_.error("Every FatFs drive slot is in use (raise CONFIG_FATFS_VOLUME_COUNT)");
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }
  ff_diskio_register_sdmmc(pdrv, &card_);
  ff_sdmmc_set_disk_status_check(pdrv, config_.disk_status_check);
  const std::string drive = fat_drive_string(pdrv);

  FATFS *fs = nullptr;
  esp_err_t err;
  {
// Plain-macro version tests (cppcheck cannot evaluate ESP_IDF_VERSION_VAL()).
#if ESP_IDF_VERSION_MAJOR > 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 3)
    esp_vfs_fat_conf_t conf{};
    conf.base_path = config_.mount_point.c_str();
    conf.fat_drive = drive.c_str();
    conf.max_files = static_cast<size_t>(config_.max_files);
#if ESP_IDF_VERSION_MAJOR >= 6
    err = esp_vfs_fat_register(&conf, &fs);
#else
    err = esp_vfs_fat_register_cfg(&conf, &fs);
#endif
#else
    err = esp_vfs_fat_register(config_.mount_point.c_str(), drive.c_str(), config_.max_files, &fs);
#endif
  }
  if (err != ESP_OK) {
    logger_.error("Could not register '{}' with the VFS: {}", config_.mount_point,
                  esp_err_to_name(err));
    ff_diskio_unregister(pdrv);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  FRESULT res = f_mount(fs, drive.c_str(), 1);
  if (res == FR_NO_FILESYSTEM || res == FR_INT_ERR) {
    if (!config_.format_if_mount_failed) {
      logger_.error("No FAT filesystem on the card (format() it, or set "
                    "format_if_mount_failed)");
      esp_vfs_fat_unregister_path(config_.mount_point.c_str());
      ff_diskio_unregister(pdrv);
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    logger_.warn("No FAT filesystem on the card; formatting it");
    pdrv_ = pdrv;
    if (!format_locked(ec)) {
      pdrv_ = kNoDrive;
      esp_vfs_fat_unregister_path(config_.mount_point.c_str());
      ff_diskio_unregister(pdrv);
      return false;
    }
    pdrv_ = kNoDrive;
    res = f_mount(fs, drive.c_str(), 1);
  }
  if (res != FR_OK) {
    logger_.error("Mounting the card failed (FatFs result {})", static_cast<int>(res));
    esp_vfs_fat_unregister_path(config_.mount_point.c_str());
    ff_diskio_unregister(pdrv);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  pdrv_ = pdrv;
  mounted_ = true;
  logger_.info("Mounted at '{}'", config_.mount_point);
  return true;
}

bool SdCard::unmount() {
  std::error_code ec;
  return unmount(ec);
}

bool SdCard::unmount(std::error_code &ec) {
  ec.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  return unmount_locked(ec);
}

bool SdCard::unmount_locked(std::error_code &ec) {
  if (!mounted_)
    return true;
  const std::string drive = fat_drive_string(pdrv_);
  f_mount(nullptr, drive.c_str(), 0);
  ff_diskio_unregister(pdrv_);
  const esp_err_t err = esp_vfs_fat_unregister_path(config_.mount_point.c_str());
  pdrv_ = kNoDrive;
  mounted_ = false;
  if (err != ESP_OK) {
    logger_.warn("Unregistering '{}' from the VFS failed: {}", config_.mount_point,
                 esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  logger_.info("Unmounted '{}'", config_.mount_point);
  return true;
}

bool SdCard::format() {
  std::error_code ec;
  return format(ec);
}

bool SdCard::format(std::error_code &ec) {
  ec.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  const bool was_mounted = mounted_;
  if (was_mounted) {
    // keep the drive registered (pdrv_) but drop the FatFs mount for f_mkfs
    const std::string drive = fat_drive_string(pdrv_);
    f_mount(nullptr, drive.c_str(), 0);
  } else {
    uint8_t pdrv = kNoDrive;
    if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == kNoDrive) {
      logger_.error("Every FatFs drive slot is in use (raise CONFIG_FATFS_VOLUME_COUNT)");
      ec = std::make_error_code(std::errc::device_or_resource_busy);
      return false;
    }
    ff_diskio_register_sdmmc(pdrv, &card_);
    pdrv_ = pdrv;
  }
  const bool ok = format_locked(ec);
  if (was_mounted) {
    // mount again on the still-registered drive; a failure here leaves the
    // volume unmounted (reported through ec)
    mounted_ = false;
    const uint8_t pdrv = pdrv_;
    pdrv_ = kNoDrive;
    esp_vfs_fat_unregister_path(config_.mount_point.c_str());
    ff_diskio_unregister(pdrv);
    std::error_code mount_ec;
    if (!mount_locked(mount_ec) && !ec)
      ec = mount_ec;
  } else {
    ff_diskio_unregister(pdrv_);
    pdrv_ = kNoDrive;
  }
  return ok && !ec;
}

bool SdCard::format_locked(std::error_code &ec) {
  // pdrv_ must be registered (not necessarily mounted) when this runs
  const std::string drive = fat_drive_string(pdrv_);
  constexpr size_t kWorkBufferSize = 4096;
  void *work = ff_memalloc(kWorkBufferSize);
  if (!work) {
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }
  // FM_ANY: FatFs picks FAT12/16/32 (or exFAT if enabled) from the card size and
  // creates an MBR partition (no FM_SFD), like ESP-IDF's own SD formatting.
  MKFS_PARM opt{};
  opt.fmt = FM_ANY;
  opt.au_size = config_.allocation_unit_size;
  logger_.info("Formatting the card (allocation unit {} bytes)", config_.allocation_unit_size);
  const FRESULT res = f_mkfs(drive.c_str(), &opt, work, kWorkBufferSize);
  ff_memfree(work);
  if (res != FR_OK) {
    logger_.error("Formatting failed (FatFs result {})", static_cast<int>(res));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  return true;
}

bool SdCard::deinitialize() {
  std::error_code ec;
  return deinitialize(ec);
}

bool SdCard::deinitialize(std::error_code &ec) {
  ec.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_)
    return true;
  std::error_code unmount_ec;
  unmount_locked(unmount_ec);
  deinit_host();
  initialized_ = false;
  std::memset(&card_, 0, sizeof(card_));
  if (unmount_ec)
    ec = unmount_ec;
  logger_.info("Deinitialized");
  return !ec;
}

bool SdCard::is_initialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

bool SdCard::is_mounted() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mounted_;
}

SdCard::Interface SdCard::interface() const {
  return std::holds_alternative<SpiConfig>(config_.interface) ? Interface::Spi : Interface::Sdmmc;
}

sdmmc_card_t *SdCard::card() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // the card lives in this object; the pointer is stable for the object's life
  return initialized_ ? const_cast<sdmmc_card_t *>(&card_) : nullptr;
}

std::optional<SdCard::CardInfo> SdCard::card_info() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_)
    return std::nullopt;
  CardInfo info;
  info.name.assign(card_.cid.name, strnlen(card_.cid.name, sizeof(card_.cid.name)));
  info.sector_size = static_cast<uint32_t>(card_.csd.sector_size);
  info.sector_count = static_cast<uint32_t>(card_.csd.capacity);
  info.capacity_bytes = static_cast<uint64_t>(info.sector_count) * info.sector_size;
  info.frequency_khz = static_cast<uint32_t>(card_.real_freq_khz);
  info.bus_width = static_cast<uint8_t>(1u << card_.log_bus_width);
  info.high_capacity = (card_.ocr & SD_OCR_SDHC_CAP) != 0;
  info.is_mmc = card_.is_mmc != 0;
  info.interface = interface();
  return info;
}

std::optional<SdCard::VolumeInfo> SdCard::volume_info() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!mounted_)
    return std::nullopt;
  VolumeInfo info;
  if (esp_vfs_fat_info(config_.mount_point.c_str(), &info.total_bytes, &info.free_bytes) != ESP_OK)
    return std::nullopt;
  return info;
}

void SdCard::print_info(FILE *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_)
    sdmmc_card_print_info(out, &card_);
}

} // namespace espp
