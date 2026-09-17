#pragma once

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <variant>

#include <driver/gpio.h>
#include <driver/sdspi_host.h>
#include <driver/spi_master.h>
#include <sdmmc_cmd.h>
#include <soc/soc_caps.h>
#if SOC_SDMMC_HOST_SUPPORTED
#include <driver/sdmmc_host.h>
#endif

#include "base_component.hpp"

namespace espp {

/**
 * @brief SD / microSD card over SDSPI or SDMMC (SDIO), with card
 *        initialization and FAT mounting as two separate steps.
 *
 * @details ESP-IDF's convenience functions (`esp_vfs_fat_sdspi_mount()` /
 * `esp_vfs_fat_sdmmc_mount()`) initialize the card and mount its FAT volume in
 * one call, and own the card for as long as it is mounted. That is fine for a
 * board that only ever reads its own card, but not when something else needs the
 * raw card: USB mass storage (`espp::UsbDevice`'s MSC function) hands the card to
 * a PC, which must not happen while the firmware has the volume mounted.
 *
 * `SdCard` therefore separates the two:
 *
 * - `initialize()` brings up the host (an SPI bus device, or an SDMMC slot) and
 *   probes the card: afterwards `card()` is a valid `sdmmc_card_t` usable for raw
 *   sector access or for handing to another owner. By default it also mounts.
 * - `mount()` / `unmount()` register / unregister the card's FAT volume at
 *   `Config::mount_point`, so the card can move between the application's VFS and
 *   another user (USB host) any number of times without re-probing it.
 *
 * Both interfaces are configured through one `Config` and selected with a
 * `std::variant`:
 *
 * - `SpiConfig`: the card on an SPI bus (any target). The bus may already be
 *   initialized by the application / BSP (shared with a display, a radio, ...) or
 *   the component can initialize and later free it.
 * - `SdmmcConfig`: the dedicated SDMMC peripheral (ESP32, ESP32-S3, ESP32-P4), 1-
 *   or 4-bit, with the pins routed through the GPIO matrix on targets that support
 *   it, and an optional on-chip LDO channel powering the card (ESP32-P4).
 *
 * \section sdcard_ex1 SdCard Example
 * \snippet sdcard_example.cpp sdcard example
 */
class SdCard : public BaseComponent {
public:
  /// @brief The card is attached to an SPI bus (SDSPI). Works on every target.
  struct SpiConfig {
    spi_host_device_t host{SPI2_HOST}; /**< SPI peripheral the card is on. */
    gpio_num_t cs{GPIO_NUM_NC};        /**< Card chip-select pin. */
    /** Initialize the SPI bus (mosi / miso / sclk below) in initialize() and free it
     *  in deinitialize(). Leave false when the application or BSP already owns
     *  the bus (e.g. it is shared with a display), in which case only `host` and
     *  `cs` are used. */
    bool initialize_bus{false};
    gpio_num_t mosi{GPIO_NUM_NC}; /**< Bus MOSI (only with initialize_bus). */
    gpio_num_t miso{GPIO_NUM_NC}; /**< Bus MISO (only with initialize_bus). */
    gpio_num_t sclk{GPIO_NUM_NC}; /**< Bus SCLK (only with initialize_bus). */
    /** Largest transfer the bus will carry, in bytes (only with initialize_bus).
     *  Multi-sector reads need at least the sector size (512). */
    int max_transfer_size{4092};
    /** SPI clock while talking to the card, in kHz. SDSPI supports 400 kHz up
     *  to 20 MHz (SDMMC_FREQ_DEFAULT). */
    int frequency_khz{SDMMC_FREQ_DEFAULT};
    gpio_num_t card_detect{GPIO_NUM_NC};   /**< Card-detect input, if wired. */
    gpio_num_t write_protect{GPIO_NUM_NC}; /**< Write-protect input, if wired. */
  };

  /// @brief The card is on the SDMMC (SDIO) peripheral: ESP32, ESP32-S3, ESP32-P4.
  struct SdmmcConfig {
    /** SDMMC slot. ESP32: slot 0 (8-bit capable, shares pins with flash on some
     *  modules) or slot 1 (4-bit, the usual choice); ESP32-S3 / -P4: any slot,
     *  pins are routed through the GPIO matrix. */
    int slot{1};
    uint8_t bus_width{4}; /**< Data bus width: 1 or 4. */
    /** Pins (targets with SOC_SDMMC_USE_GPIO_MATRIX only, e.g. ESP32-S3 / -P4;
     *  the ESP32 uses its fixed slot pins and ignores these). GPIO_NUM_NC keeps
     *  the slot's default pin. d1..d3 are unused with bus_width 1. */
    gpio_num_t clk{GPIO_NUM_NC};
    gpio_num_t cmd{GPIO_NUM_NC};
    gpio_num_t d0{GPIO_NUM_NC};
    gpio_num_t d1{GPIO_NUM_NC};
    gpio_num_t d2{GPIO_NUM_NC};
    gpio_num_t d3{GPIO_NUM_NC};
    /** Bus clock in kHz: SDMMC_FREQ_DEFAULT (20 MHz), SDMMC_FREQ_HIGHSPEED
     *  (40 MHz), or SDMMC_FREQ_PROBING (400 kHz) for marginal wiring. */
    int frequency_khz{SDMMC_FREQ_HIGHSPEED};
    gpio_num_t card_detect{GPIO_NUM_NC};   /**< Card-detect input, if wired. */
    gpio_num_t write_protect{GPIO_NUM_NC}; /**< Write-protect input, if wired. */
    /** On-chip LDO channel that powers the card's IO rail, or -1 if the card is
     *  powered externally. The ESP32-P4 feeds the SD pads from LDO_VO4 (channel
     *  4): without it the bus floats and the card never answers. */
    int ldo_channel{-1};
  };

  /// @brief Configuration for the SdCard.
  struct Config {
    /** Which interface the card is on and how it is wired. */
    std::variant<SpiConfig, SdmmcConfig> interface {
      SpiConfig {}
    };
    std::string mount_point{"/sdcard"}; /**< VFS path the FAT volume is mounted at. */
    /** Mount the FAT volume at the end of initialize(). Leave false when the card
     *  is first going elsewhere (e.g. to a USB host) and call mount() later. */
    bool mount_on_initialize{true};
    /** If the card has no FAT filesystem, create one when mounting (this erases
     *  whatever is on the card). Off by default: mount() then fails with
     *  `std::errc::no_such_device` and format() is available. */
    bool format_if_mount_failed{false};
    int max_files{5}; /**< Files the application may keep open at once. */
    /** FAT allocation unit (cluster) size in bytes used when the card is formatted;
     *  0 = FatFs picks one from the card size. Larger clusters make big files
     *  faster and small files wasteful. */
    size_t allocation_unit_size{16 * 1024};
    /** Ask the card for its status before every FAT operation, so a card removed
     *  while mounted is noticed instead of returning stale data; costs a command
     *  per operation. */
    bool disk_status_check{false};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /// @brief Which interface a configured card uses.
  enum class Interface : uint8_t { Spi, Sdmmc };

  /// @brief What the card reported about itself at initialize().
  struct CardInfo {
    std::string name;                       ///< Product name from the card's CID register.
    uint64_t capacity_bytes{0};             ///< Total capacity.
    uint32_t sector_size{0};                ///< Bytes per sector (512 for every SD card).
    uint32_t sector_count{0};               ///< Number of sectors.
    uint32_t frequency_khz{0};              ///< Bus clock actually in use.
    uint8_t bus_width{1};                   ///< Data lines in use (SDSPI: 1).
    bool high_capacity{false};              ///< SDHC / SDXC (block addressing).
    bool is_mmc{false};                     ///< An (e)MMC device rather than an SD card.
    Interface interface { Interface::Spi }; ///< The interface it is on.
  };

  /// @brief Space on the mounted FAT volume.
  struct VolumeInfo {
    uint64_t total_bytes{0}; ///< Volume size.
    uint64_t free_bytes{0};  ///< Unallocated space.
  };

  /**
   * @brief Construct the component. Does not touch hardware until initialize().
   * @param config Configuration.
   */
  explicit SdCard(const Config &config);

  /// @brief Unmounts the volume (if mounted) and releases the card, host and,
  ///        when the component initialized it, the SPI bus.
  ~SdCard();

  SdCard(const SdCard &) = delete;
  SdCard &operator=(const SdCard &) = delete;

  /**
   * @brief Bring up the host (SPI device / SDMMC slot, LDO), probe the card and,
   *        with Config::mount_on_initialize, mount its FAT volume.
   * @param[out] ec Set on failure: invalid configuration (`invalid_argument`),
   *        the host could not be initialized (`io_error`), no card answered
   *        (`no_such_device` -- check the card, the wiring and the pull-ups), or a
   *        mount failure (see mount()). Nothing stays initialized on failure.
   * @return true on success.
   */
  bool initialize(std::error_code &ec);

  /// @brief Convenience overload of initialize() that ignores errors.
  bool initialize();

  /**
   * @brief Mount the card's FAT volume at Config::mount_point.
   * @param[out] ec Set on failure: not initialized (`not_connected`), every FatFs
   *        drive slot in use (`device_or_resource_busy` -- raise
   *        CONFIG_FATFS_VOLUME_COUNT), no FAT filesystem on the card and
   *        Config::format_if_mount_failed off (`no_such_device` -- see format()),
   *        or the mount / VFS registration failed (`io_error`).
   * @return true if the volume is mounted (also when it already was).
   */
  bool mount(std::error_code &ec);

  /// @brief Convenience overload of mount() that ignores errors.
  bool mount();

  /**
   * @brief Unmount the FAT volume, releasing Config::mount_point. Files still
   *        open there become invalid. The card stays initialized: card() remains
   *        valid and mount() may be called again.
   * @param[out] ec Set on failure (`not_connected` if not initialized).
   * @return true if the volume is unmounted (also when it already was).
   */
  bool unmount(std::error_code &ec);

  /// @brief Convenience overload of unmount() that ignores errors.
  bool unmount();

  /**
   * @brief Create a fresh FAT filesystem on the card (erasing everything on it),
   *        using Config::allocation_unit_size. The volume is unmounted first if
   *        it was mounted, and mounted again afterwards.
   * @param[out] ec Set on failure (`not_connected` if not initialized, else
   *        `io_error`).
   * @return true if the card was formatted.
   */
  bool format(std::error_code &ec);

  /// @brief Convenience overload of format() that ignores errors.
  bool format();

  /**
   * @brief Release everything: unmount, detach the card from the host, delete
   *        the LDO handle and free the SPI bus if the component initialized it.
   *        The destructor calls this.
   * @param[out] ec Set on failure (the object is still deinitialized).
   * @return true on success.
   */
  bool deinitialize(std::error_code &ec);

  /// @brief Convenience overload of deinitialize() that ignores errors.
  bool deinitialize();

  /// @brief Whether initialize() succeeded (the card is probed and card() is valid).
  bool is_initialized() const;

  /// @brief Whether the FAT volume is currently mounted at mount_point().
  bool is_mounted() const;

  /// @brief The interface the card is configured on.
  Interface interface() const;

  /// @brief The VFS path the volume is (or would be) mounted at.
  const std::string &mount_point() const { return config_.mount_point; }

  /**
   * @brief The initialized card, for raw sector access or to hand to another
   *        owner (e.g. `espp::UsbDevice::MscMedium::sd_card`). Valid from a
   *        successful initialize() until deinitialize(); the SdCard keeps owning
   *        it. nullptr when not initialized.
   * @note Whoever uses the card directly must do so while the volume is NOT
   *       mounted here (unmount() first): FatFs and a raw writer must not share
   *       the card.
   */
  sdmmc_card_t *card() const;

  /// @brief What the card reported at initialize(); nullopt if not initialized.
  std::optional<CardInfo> card_info() const;

  /// @brief Total / free space on the mounted volume; nullopt if not mounted.
  std::optional<VolumeInfo> volume_info() const;

  /// @brief Print the card's properties (what `sdmmc_card_print_info()` prints).
  /// @param out Stream to print to (default stdout).
  void print_info(FILE *out = stdout) const;

protected:
  bool init_host(std::error_code &ec);
  void deinit_host();
  bool mount_locked(std::error_code &ec);
  bool unmount_locked(std::error_code &ec);
  bool format_locked(std::error_code &ec);

  Config config_;
  mutable std::mutex mutex_;
  bool initialized_{false};
  bool mounted_{false};
  bool bus_initialized_{false}; // we initialized the SPI bus (SpiConfig::initialize_bus)
  sdmmc_host_t host_{};         // host with the (SDSPI: device handle) slot filled in
  sdmmc_card_t card_{};         // the probed card (owned here)
  uint8_t pdrv_{0xFF};          // FatFs drive number while mounted
  void *ldo_handle_{nullptr};   // sd_pwr_ctrl_handle_t while an LDO channel is in use
};

} // namespace espp
