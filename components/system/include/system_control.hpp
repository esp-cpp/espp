#pragma once

#include <chrono>
#include <system_error>
#include <thread>

#include "sdkconfig.h"

#include "esp_system.h"
#include "soc/soc.h"

// The "force download boot" bit lives in a different always-on register on
// each chip family; ESP-IDF's own ROM USB console (esp_usb_cdc_rom_console)
// sets it the same way before restarting.
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 ||         \
    CONFIG_IDF_TARGET_ESP32C2
#include "soc/rtc_cntl_reg.h"
#define ESPP_SYSTEM_DOWNLOAD_BOOT_RTC_CNTL 1
#elif CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32H2 || CONFIG_IDF_TARGET_ESP32C5 ||       \
    CONFIG_IDF_TARGET_ESP32C61 || CONFIG_IDF_TARGET_ESP32H21
#include "soc/lp_aon_reg.h"
#define ESPP_SYSTEM_DOWNLOAD_BOOT_LP_AON 1
#elif CONFIG_IDF_TARGET_ESP32P4
#include "soc/lp_system_reg.h"
#define ESPP_SYSTEM_DOWNLOAD_BOOT_LP_SYSTEM 1
#endif
// The ROM USB persistence calls (usb_dc_prepare_persist() +
// chip_usb_set_persist_flags()) exist on the S2 / S3 only, whose ROM has a USB
// CDC / DFU device of its own; they are used only when a caller opts in.
#if CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/usb/chip_usb_dw_wrapper.h"
#include "esp32s2/rom/usb/usb_dc.h"
#include "esp32s2/rom/usb/usb_persist.h"
#define ESPP_SYSTEM_USB_PERSIST 1
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/usb/chip_usb_dw_wrapper.h"
#include "esp32s3/rom/usb/usb_dc.h"
#include "esp32s3/rom/usb/usb_persist.h"
#define ESPP_SYSTEM_USB_PERSIST 1
#endif

namespace espp {

/// Options for SystemControl::reboot_to_bootloader() (a namespace-scope type
/// so it is complete where the functions default it).
struct SystemBootloaderOptions {
  /// Keep the USB peripheral's state across the reset (ESP32-S2 / -S3 ROM
  /// only; ignored elsewhere). ONLY for applications whose USB device is
  /// ROM-CDC/DFU-compatible (see the SystemControl notes); off by default,
  /// letting the ROM re-enumerate on its own.
  bool usb_persist{false};
};

/**
 * @brief Restart control: a plain reboot, and a reboot into the ROM
 *        bootloader's download (serial flashing) mode.
 *
 * reboot_to_bootloader() sets the chip's "force download boot" flag in its
 * always-on register and restarts, so the next boot stays in the ROM
 * download mode instead of running the app -- exactly what holding the BOOT
 * strap during a reset does, without touching a button. The device then
 * re-enumerates as the ROM's own flashing interface: the USB CDC / DFU device
 * on the ESP32-S2 / -S3 native USB port, or USB-Serial-JTAG on the ESP32-C3 /
 * -C6 / -H2 / -C5 / -C61 / -H21 / -P4 -- so `esptool` / `idf.py flash` can
 * program it. On the classic ESP32 there is no software path (only the GPIO0
 * strap): bootloader_reboot_supported() is false and reboot_to_bootloader()
 * fails with operation_not_supported.
 *
 * **USB persistence (S2 / S3, opt-in)**: by default the reset tears the USB
 * connection down and the ROM enumerates its CDC / DFU device afresh, which
 * works for any application. The ROM can instead keep the USB peripheral's
 * state across the reset (BootloaderOptions::usb_persist), so the host sees
 * no re-plug -- but the ROM only expects that from an application whose USB
 * device is ROM-CDC/DFU-compatible (the same descriptors the ROM exposes, as
 * ESP-IDF's ROM USB console has); a TinyUSB composite device (the espp
 * examples: vendor + CDC) has different descriptors, and persisting it can
 * leave the host with a stale enumeration the bootloader cannot serve. Leave
 * it off unless the application runs on the ROM USB console driver.
 *
 * Both functions restart immediately; use the delayed variants (or
 * espp::SystemService, which replies before restarting) when a reply must
 * leave the transport first.
 */
class SystemControl {
public:
  /// Options for the reboot into download mode (see SystemBootloaderOptions).
  using BootloaderOptions = SystemBootloaderOptions;

  /// @brief Whether reboot_to_bootloader() is implemented for this chip.
  static constexpr bool bootloader_reboot_supported() {
#if ESPP_SYSTEM_DOWNLOAD_BOOT_RTC_CNTL || ESPP_SYSTEM_DOWNLOAD_BOOT_LP_AON ||                      \
    ESPP_SYSTEM_DOWNLOAD_BOOT_LP_SYSTEM
    return true;
#else
    return false;
#endif
  }

  /// @brief Restart the chip (esp_restart()); does not return.
  [[noreturn]] static void reboot() { esp_restart(); }

  /// @brief Restart into the ROM bootloader's download mode.
  /// @param ec Set to operation_not_supported on chips without a software path
  ///        (classic ESP32); then returns false without restarting.
  /// @param options See BootloaderOptions (USB persistence is opt-in).
  /// @return Does not return on success; false on failure.
  static bool reboot_to_bootloader(std::error_code &ec, const BootloaderOptions &options = {}) {
    ec.clear();
    if (!bootloader_reboot_supported()) {
      ec = std::make_error_code(std::errc::operation_not_supported);
      return false;
    }
    arm_download_boot(options);
    esp_restart();
    return true; // not reached
  }

  /// @brief Restart after a delay, from a detached thread, so the caller can
  ///        finish (e.g. send a reply) first. Returns immediately.
  static void reboot_after(std::chrono::milliseconds delay) {
    std::thread([delay]() {
      std::this_thread::sleep_for(delay);
      esp_restart();
    }).detach();
  }

  /// @brief Restart into download mode after a delay, from a detached thread.
  ///        Returns immediately; false (nothing scheduled) if unsupported.
  static bool reboot_to_bootloader_after(std::chrono::milliseconds delay, std::error_code &ec,
                                         const BootloaderOptions &options = {}) {
    ec.clear();
    if (!bootloader_reboot_supported()) {
      ec = std::make_error_code(std::errc::operation_not_supported);
      return false;
    }
    std::thread([delay, options]() {
      std::this_thread::sleep_for(delay);
      arm_download_boot(options);
      esp_restart();
    }).detach();
    return true;
  }

private:
  /// Set the chip's force-download-boot flag (survives the reset that follows).
  static void arm_download_boot(const BootloaderOptions &options) {
#if ESPP_SYSTEM_USB_PERSIST
    if (options.usb_persist) {
      // Keep the ROM USB device attached across the reset, the way ESP-IDF's
      // ROM USB console does before its own reboot-to-bootloader: park the
      // peripheral (usb_dc_prepare_persist(), "reboot soon after") and set the
      // persist flag the ROM reads on the next boot. Opt-in only: see the
      // class notes.
      usb_dc_prepare_persist();
      chip_usb_set_persist_flags(USBDC_PERSIST_ENA);
    }
#else
    (void)options;
#endif
#if ESPP_SYSTEM_DOWNLOAD_BOOT_RTC_CNTL
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
#elif ESPP_SYSTEM_DOWNLOAD_BOOT_LP_AON
    // a 1-bit flag on most chips; a 2-bit field on the C5 where 1 = "force
    // download boot 0 (UART / USB)" -- writing the field value 1 covers both
    REG_SET_FIELD(LP_AON_SYS_CFG_REG, LP_AON_FORCE_DOWNLOAD_BOOT, 1);
#elif ESPP_SYSTEM_DOWNLOAD_BOOT_LP_SYSTEM
    REG_SET_FIELD(LP_SYSTEM_REG_SYS_CTRL_REG, LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT, 1);
#endif
  }
};

} // namespace espp
