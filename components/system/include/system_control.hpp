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
#if CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/usb/chip_usb_dw_wrapper.h"
#include "esp32s2/rom/usb/usb_persist.h"
#define ESPP_SYSTEM_USB_PERSIST 1
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/usb/chip_usb_dw_wrapper.h"
#include "esp32s3/rom/usb/usb_persist.h"
#define ESPP_SYSTEM_USB_PERSIST 1
#endif

namespace espp {

/**
 * @brief Restart control: a plain reboot, and a reboot into the ROM
 *        bootloader's download (serial flashing) mode.
 *
 * reboot_to_bootloader() sets the chip's "force download boot" flag in its
 * always-on register and restarts, so the next boot stays in the ROM
 * download mode instead of running the app -- exactly what holding the BOOT
 * strap during a reset does, without touching a button. The device then
 * re-enumerates as the ROM's own flashing interface: the USB CDC / DFU device
 * on the ESP32-S2 / -S3 native USB port (the ROM's USB stack is kept
 * persistent across the reset), or USB-Serial-JTAG on the ESP32-C3 / -C6 /
 * -H2 / -C5 / -C61 / -H21 / -P4 -- so `esptool` / `idf.py flash` can program
 * it. On the classic ESP32 there is no software path (only the GPIO0 strap):
 * bootloader_reboot_supported() is false and reboot_to_bootloader() fails
 * with operation_not_supported.
 *
 * Both functions restart immediately; use the delayed variants (or
 * espp::SystemService, which replies before restarting) when a reply must
 * leave the transport first.
 */
class SystemControl {
public:
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
  /// @return Does not return on success; false on failure.
  static bool reboot_to_bootloader(std::error_code &ec) {
    ec.clear();
    if (!bootloader_reboot_supported()) {
      ec = std::make_error_code(std::errc::operation_not_supported);
      return false;
    }
    arm_download_boot();
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
  static bool reboot_to_bootloader_after(std::chrono::milliseconds delay, std::error_code &ec) {
    ec.clear();
    if (!bootloader_reboot_supported()) {
      ec = std::make_error_code(std::errc::operation_not_supported);
      return false;
    }
    std::thread([delay]() {
      std::this_thread::sleep_for(delay);
      arm_download_boot();
      esp_restart();
    }).detach();
    return true;
  }

private:
  /// Set the chip's force-download-boot flag (survives the reset that follows).
  static void arm_download_boot() {
#if ESPP_SYSTEM_USB_PERSIST
    // keep the ROM's USB device attached across the reset so the host sees the
    // download-mode CDC / DFU interface without a full re-plug
    chip_usb_set_persist_flags(USBDC_PERSIST_ENA);
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
