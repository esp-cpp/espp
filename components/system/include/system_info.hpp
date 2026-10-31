#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>

#include "sdkconfig.h"

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_clk_tree.h"
#include "esp_flash.h"
#include "esp_idf_version.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#include "format.hpp"

namespace espp {

/**
 * @brief Static accessors for the identity and status of the running system:
 *        the chip, the ESP-IDF version, the application description (project
 *        name, version, build date / time, ELF SHA-256), the running / boot
 *        partitions and OTA state, the reset reason, uptime, base MAC, flash
 *        and PSRAM sizes, CPU frequency and heap figures.
 *
 * Everything is a thin, allocation-light wrapper over the corresponding
 * ESP-IDF call so it can be used from any task. collect() gathers it all into
 * one Snapshot (what espp::SystemService reports to a host) and to_string()
 * renders a human-readable summary for logs.
 *
 * \section system_info_ex1 SystemInfo Example
 * \snippet system_example.cpp system_example
 */
class SystemInfo {
public:
  /// Everything collect() gathers.
  struct Snapshot {
    std::string chip_model;               ///< e.g. "ESP32-S3"
    uint16_t chip_revision{0};            ///< MXX: major * 100 + minor
    uint8_t cores{0};                     ///< CPU core count
    uint32_t chip_features{0};            ///< CHIP_FEATURE_* bitmask
    std::string idf_version;              ///< e.g. "v6.1"
    std::string project_name;             ///< CMake project name
    std::string app_version;              ///< PROJECT_VER
    std::string build_date;               ///< compile date
    std::string build_time;               ///< compile time
    std::array<uint8_t, 32> elf_sha256{}; ///< SHA-256 of the application ELF
    std::string running_partition;        ///< label of the partition the app runs from
    std::string boot_partition;           ///< label of the partition the bootloader will boot next
    uint8_t ota_state{0xFF};      ///< esp_ota_img_states_t of the running partition; 0xFF = n/a
    uint8_t reset_reason{0};      ///< esp_reset_reason_t
    uint64_t uptime_ms{0};        ///< time since boot
    std::array<uint8_t, 6> mac{}; ///< base MAC address
    uint32_t flash_size{0};       ///< bytes
    uint32_t psram_size{0};       ///< bytes (0 = none / disabled)
    uint32_t cpu_mhz{0};          ///< current CPU frequency
    uint32_t free_heap{0};        ///< bytes
    uint32_t min_free_heap{0};    ///< bytes, lowest since boot
  };

  /// @brief Chip model name for an esp_chip_model_t ("ESP32-S3", ...).
  static const char *chip_model_name(esp_chip_model_t model) {
    // Compared by value rather than by enumerator so this compiles against
    // ESP-IDF releases that predate the newer chips.
    switch (static_cast<int>(model)) {
    case 1:
      return "ESP32";
    case 2:
      return "ESP32-S2";
    case 9:
      return "ESP32-S3";
    case 5:
      return "ESP32-C3";
    case 12:
      return "ESP32-C2";
    case 13:
      return "ESP32-C6";
    case 16:
      return "ESP32-H2";
    case 18:
      return "ESP32-P4";
    case 20:
      return "ESP32-C61";
    case 23:
      return "ESP32-C5";
    case 25:
      return "ESP32-H21";
    case 28:
      return "ESP32-H4";
    case 32:
      return "ESP32-S31";
    default:
      return "ESP32 (unknown model)";
    }
  }

  /// @brief Human-readable name for an esp_reset_reason_t.
  static const char *reset_reason_name(esp_reset_reason_t reason) {
    switch (static_cast<int>(reason)) {
    case 0:
      return "unknown";
    case 1:
      return "power-on";
    case 2:
      return "external pin";
    case 3:
      return "software (esp_restart)";
    case 4:
      return "panic";
    case 5:
      return "interrupt watchdog";
    case 6:
      return "task watchdog";
    case 7:
      return "other watchdog";
    case 8:
      return "deep-sleep wake";
    case 9:
      return "brownout";
    case 10:
      return "SDIO";
    case 11:
      return "USB";
    case 12:
      return "JTAG";
    case 13:
      return "efuse error";
    case 14:
      return "power glitch";
    case 15:
      return "CPU lockup";
    default:
      return "unknown";
    }
  }

  /// @brief Human-readable name for an OTA image state byte (Snapshot::ota_state).
  static const char *ota_state_name(uint8_t state) {
    switch (state) {
    case 0:
      return "new";
    case 1:
      return "pending verify";
    case 2:
      return "valid";
    case 3:
      return "invalid";
    case 4:
      return "aborted";
    case 0xFF:
      return "n/a";
    default:
      return "undefined";
    }
  }

  /// @brief Chip information (model, revision, cores, features).
  static esp_chip_info_t chip_info() {
    esp_chip_info_t info{};
    esp_chip_info(&info);
    return info;
  }

  /// @brief The ESP-IDF version string the app was built with.
  static std::string idf_version() { return esp_get_idf_version(); }

  /// @brief The application description embedded in the running image.
  static const esp_app_desc_t &app_description() { return *esp_app_get_description(); }

  /// @brief Label of the partition the running app was loaded from ("" if unknown).
  static std::string running_partition() {
    const esp_partition_t *p = esp_ota_get_running_partition();
    return p ? p->label : "";
  }

  /// @brief Label of the partition the bootloader will boot next ("" if unknown).
  static std::string boot_partition() {
    const esp_partition_t *p = esp_ota_get_boot_partition();
    return p ? p->label : "";
  }

  /// @brief OTA image state of the running partition (esp_ota_img_states_t as a
  ///        byte; 0xFF when the app runs from a factory partition or the state
  ///        cannot be read).
  static uint8_t ota_state() {
    const esp_partition_t *p = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (!p || esp_ota_get_state_partition(p, &state) != ESP_OK)
      return 0xFF;
    return state == ESP_OTA_IMG_UNDEFINED ? 0xFE : static_cast<uint8_t>(state);
  }

  /// @brief Why the chip last reset.
  static esp_reset_reason_t reset_reason() { return esp_reset_reason(); }

  /// @brief Milliseconds since boot.
  static uint64_t uptime_ms() { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }

  /// @brief The chip's base (factory-programmed) MAC address.
  static std::array<uint8_t, 6> base_mac() {
    std::array<uint8_t, 6> mac{};
    esp_efuse_mac_get_default(mac.data());
    return mac;
  }

  /// @brief Size of the main SPI flash in bytes (0 if it cannot be read).
  static uint32_t flash_size() {
    uint32_t size = 0;
    if (esp_flash_get_size(nullptr, &size) != ESP_OK)
      return 0;
    return size;
  }

  /// @brief Size of the PSRAM in bytes (0 without PSRAM / with CONFIG_SPIRAM off).
  static uint32_t psram_size() {
#if CONFIG_SPIRAM
    return static_cast<uint32_t>(esp_psram_get_size());
#else
    return 0;
#endif
  }

  /// @brief Current CPU frequency in MHz.
  static uint32_t cpu_mhz() {
    uint32_t hz = 0;
    if (esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                     &hz) != ESP_OK ||
        hz == 0)
      return CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    return hz / 1000000u;
  }

  /// @brief Free heap in bytes (default capabilities).
  static uint32_t free_heap() { return esp_get_free_heap_size(); }

  /// @brief Lowest free heap since boot, in bytes.
  static uint32_t min_free_heap() { return esp_get_minimum_free_heap_size(); }

  /// @brief Gather everything into one Snapshot.
  static Snapshot collect() {
    Snapshot s;
    const esp_chip_info_t chip = chip_info();
    s.chip_model = chip_model_name(chip.model);
    s.chip_revision = chip.revision;
    s.cores = chip.cores;
    s.chip_features = chip.features;
    s.idf_version = idf_version();
    const esp_app_desc_t &app = app_description();
    s.project_name =
        std::string(app.project_name, strnlen(app.project_name, sizeof(app.project_name)));
    s.app_version = std::string(app.version, strnlen(app.version, sizeof(app.version)));
    s.build_date = std::string(app.date, strnlen(app.date, sizeof(app.date)));
    s.build_time = std::string(app.time, strnlen(app.time, sizeof(app.time)));
    std::memcpy(s.elf_sha256.data(), app.app_elf_sha256, s.elf_sha256.size());
    s.running_partition = running_partition();
    s.boot_partition = boot_partition();
    s.ota_state = ota_state();
    s.reset_reason = static_cast<uint8_t>(reset_reason());
    s.uptime_ms = uptime_ms();
    s.mac = base_mac();
    s.flash_size = flash_size();
    s.psram_size = psram_size();
    s.cpu_mhz = cpu_mhz();
    s.free_heap = free_heap();
    s.min_free_heap = min_free_heap();
    return s;
  }

  /// @brief A multi-line human-readable summary of a Snapshot.
  static std::string to_string(const Snapshot &s) {
    return fmt::format(
        "{} rev {}.{} ({} core{}), ESP-IDF {}\n"
        "app: {} {} built {} {}\n"
        "partition: running '{}', boot '{}', OTA state {}\n"
        "reset: {}; uptime {} ms; MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\n"
        "flash {} KiB, PSRAM {} KiB, CPU {} MHz, heap free {} (min {})",
        s.chip_model, s.chip_revision / 100, s.chip_revision % 100, s.cores,
        s.cores == 1 ? "" : "s", s.idf_version, s.project_name, s.app_version, s.build_date,
        s.build_time, s.running_partition, s.boot_partition, ota_state_name(s.ota_state),
        reset_reason_name(static_cast<esp_reset_reason_t>(s.reset_reason)), s.uptime_ms, s.mac[0],
        s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5], s.flash_size / 1024, s.psram_size / 1024,
        s.cpu_mhz, s.free_heap, s.min_free_heap);
  }

  /// @brief Collect and format in one call (for a boot banner).
  static std::string to_string() { return to_string(collect()); }
};

} // namespace espp
