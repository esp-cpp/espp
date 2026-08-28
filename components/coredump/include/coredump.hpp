#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <system_error>

#include "sdkconfig.h"

#include "esp_core_dump.h"
#include "esp_partition.h"
#include "esp_system.h"

#include "base_component.hpp"
#include "format.hpp"

namespace espp {

/**
 * @brief Idiomatic espp access to the ESP-IDF flash core dump.
 *
 * Wraps the `espcoredump` component's flash APIs
 * (`CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH` + a `coredump` data partition) in a
 * no-exceptions, `std::error_code`-reporting espp API:
 *
 * - **Crash detection & summary**: `has_core_dump()`, `summary()` (the raw
 *   `esp_core_dump_summary_t`) and `format_report()` — a ready-to-print /
 *   ready-to-transmit text report with the reset reason, crashed task, PC,
 *   raw backtrace addresses (Xtensa) or a captured stack dump (RISC-V), and
 *   the exact `addr2line` command line (with the right toolchain prefix for
 *   the build target) to decode them. Abnormal resets that do NOT produce a
 *   core dump (brownout, interrupt / task watchdog) are still reported with a
 *   short hint (e.g. brownout → check power).
 * - **Raw image access**: `image_size()`, `read_image(offset, out, ec)` and
 *   `erase(ec)` for downloading the complete core-dump image (the flash
 *   header + ELF core file + checksum) over any transport, so host tools
 *   (`espcoredump.py`, gdb, or the espp core-dump web console) can do the
 *   full offline analysis.
 *
 * The class holds no session state and performs no allocation beyond the
 * returned strings; all methods are safe to call whether or not a core dump
 * is present. When `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH` is disabled, the
 * dump-related methods degrade gracefully (no dump present) and
 * `format_report()` still reports the reset reason.
 *
 * For serving this information over a byte-stream transport (USB vendor /
 * WebUSB, CDC / Web Serial, sockets) see espp::CoreDumpService
 * (`coredump_service.hpp`) and the browser web app
 * `web/coredump_console.html`.
 *
 * \section coredump_ex1 CoreDump Example
 * \snippet coredump_example.cpp coredump_example
 */
class CoreDump : public BaseComponent {
public:
  /// Configuration for the CoreDump component.
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct the CoreDump accessor with the default configuration.
  CoreDump()
      : BaseComponent("CoreDump", espp::Logger::Verbosity::WARN) {}

  /**
   * @brief Construct the CoreDump accessor. Does not touch the flash.
   * @param config Configuration parameters.
   */
  explicit CoreDump(const Config &config)
      : BaseComponent("CoreDump", config.log_level) {}

  /// @brief Whether a valid core dump image is present in the coredump flash
  ///        partition (`esp_core_dump_image_check()`).
  bool has_core_dump() const {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    return esp_core_dump_image_check() == ESP_OK;
#else
    return false;
#endif
  }

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
  /// @brief The core dump summary (crashed task, PC, backtrace / stack dump,
  ///        app ELF SHA-256), or std::nullopt if no valid core dump is
  ///        present.
  /// @note Only available with the (default) ELF core dump format
  ///       (`CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF`).
  std::optional<esp_core_dump_summary_t> summary() const {
    if (!has_core_dump())
      return std::nullopt;
    esp_core_dump_summary_t s = {};
    if (esp_core_dump_get_summary(&s) != ESP_OK)
      return std::nullopt;
    return s;
  }
#endif

  /**
   * @brief Format a human-readable crash report for the PREVIOUS boot.
   *
   * - With a core dump present: the reset reason, panic reason (if recorded),
   *   crashed task + PC, the raw backtrace addresses (with a "(corrupted)"
   *   marker when the on-device unwind failed) on Xtensa targets or the size
   *   of the captured stack dump on RISC-V, and the exact
   *   `addr2line` command line (correct toolchain prefix for
   *   CONFIG_IDF_TARGET) to decode the addresses against the app ELF.
   * - Brownout / interrupt-watchdog / task-watchdog resets write no core
   *   dump, so the reset reason itself is reported with a short hint
   *   (brownout → check the power supply).
   * - Otherwise (normal power-on / software reset with no dump): an empty
   *   string, meaning "nothing abnormal to report".
   *
   * @return The report text ("" = clean boot history).
   */
  std::string format_report() const {
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    const char *reason_name = reset_reason_name(reset_reason);
    std::string report;
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (has_core_dump()) {
      report = fmt::format("last reset: {} ({})", reason_name, static_cast<int>(reset_reason));
#if CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
      // (the panic reason / summary readback APIs are only implemented for
      // the ELF core dump format)
      char panic_reason[128] = {};
      if (esp_core_dump_get_panic_reason(panic_reason, sizeof(panic_reason)) == ESP_OK)
        report += fmt::format("\npanic reason: {}", panic_reason);
      esp_core_dump_summary_t s = {};
      if (esp_core_dump_get_summary(&s) == ESP_OK) {
        report +=
            fmt::format("\ncrashed task: '{}' PC=0x{:08x}",
                        std::string(s.exc_task, strnlen(s.exc_task, sizeof(s.exc_task))), s.exc_pc);
#if CONFIG_IDF_TARGET_ARCH_XTENSA
        report += "\nbacktrace:";
        const auto depth = std::min<uint32_t>(s.exc_bt_info.depth, std::size(s.exc_bt_info.bt));
        for (uint32_t i = 0; i < depth; i++)
          report += fmt::format(" 0x{:08x}", s.exc_bt_info.bt[i]);
        if (s.exc_bt_info.corrupted)
          report += " (corrupted)";
#else
        // RISC-V cannot unwind on-device; the summary carries a raw stack
        // dump instead, and the full backtrace comes from the downloaded ELF
        // core dump (espcoredump.py / gdb).
        report += fmt::format("\nstack dump captured: {} bytes (backtrace requires the ELF "
                              "core dump; download it and run espcoredump.py)",
                              s.exc_bt_info.dump_size);
#endif
        report += fmt::format("\ndecode with: {}addr2line -pfiaC -e build/<app>.elf <addrs>",
                              toolchain_prefix());
      }
#endif // CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
      return report;
    }
#endif // CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    // No core dump; for these reset reasons the reason itself is the story
    // (none of them write a core dump).
    if (reset_reason == ESP_RST_BROWNOUT) {
      report = fmt::format("last reset: {} (no core dump: brownout — check the power "
                           "supply / USB cable / peripheral load)",
                           reason_name);
    } else if (reset_reason == ESP_RST_INT_WDT || reset_reason == ESP_RST_TASK_WDT ||
               reset_reason == ESP_RST_WDT) {
      report = fmt::format("last reset: {} (no core dump: watchdog reset — a task or ISR "
                           "hogged the CPU past the watchdog timeout)",
                           reason_name);
    }
    return report;
  }

  /// @brief Total size in bytes of the stored core dump image (flash header +
  ///        ELF data + checksum), or 0 if no valid core dump is present.
  size_t image_size() const {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (!has_core_dump())
      return 0;
    size_t addr = 0, size = 0;
    if (esp_core_dump_image_get(&addr, &size) != ESP_OK)
      return 0;
    return size;
#else
    return 0;
#endif
  }

  /**
   * @brief Read a chunk of the raw core dump image from flash.
   * @param offset Byte offset into the image (0 .. image_size()-1).
   * @param out Destination span; up to `out.size()` bytes are read (the span
   *        is NOT resized — reads past the end of the image fail).
   * @param[out] ec Set on failure: no core dump present (no_such_device), the
   *        requested range exceeds the image or the reported image lies
   *        outside the coredump partition (result_out_of_range), or the flash
   *        read failed (io_error).
   * @return true if `out.size()` bytes were read into @p out, false otherwise.
   */
  bool read_image(size_t offset, std::span<uint8_t> out, std::error_code &ec) const {
    ec.clear();
    if (out.empty())
      return true;
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    size_t addr = 0, size = 0;
    if (!has_core_dump() || esp_core_dump_image_get(&addr, &size) != ESP_OK) {
      logger_.error("read_image: no valid core dump image present");
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    if (offset > size || out.size() > size - offset) {
      logger_.error("read_image: range [{}, {}) exceeds image size {}", offset, offset + out.size(),
                    size);
      ec = std::make_error_code(std::errc::result_out_of_range);
      return false;
    }
    const esp_partition_t *partition = coredump_partition();
    if (partition == nullptr) {
      logger_.error("read_image: no coredump partition found");
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    // esp_core_dump_image_get() returns the absolute flash address (== the
    // partition base); read partition-relative. Defensively validate that the
    // reported image really lies within the partition before translating, so
    // an inconsistent address can never underflow the subtraction or read
    // from the wrong flash region.
    if (addr < partition->address || size > partition->size ||
        addr - partition->address > partition->size - size) {
      logger_.error("read_image: image [0x{:x}, 0x{:x}) lies outside the coredump "
                    "partition [0x{:x}, 0x{:x})",
                    addr, addr + size, partition->address, partition->address + partition->size);
      ec = std::make_error_code(std::errc::result_out_of_range);
      return false;
    }
    const size_t partition_offset = addr - partition->address;
    const esp_err_t err =
        esp_partition_read(partition, partition_offset + offset, out.data(), out.size());
    if (err != ESP_OK) {
      logger_.error("read_image: esp_partition_read at offset {} failed: {}", offset,
                    esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return true;
#else
    logger_.error("read_image: core dump to flash is not enabled "
                  "(CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH)");
    ec = std::make_error_code(std::errc::function_not_supported);
    return false;
#endif
  }

  /**
   * @brief Erase the stored core dump (`esp_core_dump_image_erase()`), so the
   *        next boot reports a clean history. Idempotent: succeeds as a no-op
   *        when no dump is present.
   * @param[out] ec Set on failure (io_error).
   * @return true on success (or no-op), false otherwise (ec is set).
   */
  bool erase(std::error_code &ec) {
    ec.clear();
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    const esp_err_t err = esp_core_dump_image_erase();
    if (err != ESP_OK) {
      logger_.error("erase: esp_core_dump_image_erase failed: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    logger_.info("core dump erased");
    return true;
#else
    return true; // nothing to erase
#endif
  }

  /// @brief The reset reason of THIS boot (`esp_reset_reason()`).
  static esp_reset_reason_t reset_reason() { return esp_reset_reason(); }

  /// @brief Human-readable name for a reset reason (e.g. "PANIC", "BROWNOUT").
  static const char *reset_reason_name(esp_reset_reason_t reason) {
    switch (reason) {
    case ESP_RST_UNKNOWN:
      return "UNKNOWN";
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXT";
    case ESP_RST_SW:
      return "SW";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    default:
      break;
    }
    // Newer IDF versions add more reasons (USB / JTAG / EFUSE / PWR_GLITCH /
    // CPU_LOCKUP); avoid depending on their enumerators so the component
    // builds across IDF versions.
    switch (static_cast<int>(reason)) {
    case 11:
      return "USB";
    case 12:
      return "JTAG";
    case 13:
      return "EFUSE";
    case 14:
      return "PWR_GLITCH";
    case 15:
      return "CPU_LOCKUP";
    default:
      return "?";
    }
  }

  /// @brief The GNU toolchain binary prefix for the build target (e.g.
  ///        "xtensa-esp32s3-elf-" or "riscv32-esp-elf-"), for composing
  ///        addr2line / gdb command lines in reports and host tools.
  static constexpr const char *toolchain_prefix() {
#if CONFIG_IDF_TARGET_ARCH_RISCV
    return "riscv32-esp-elf-";
#else
    // cppcheck-suppress unknownMacro // CONFIG_IDF_TARGET is a Kconfig string
    // macro ("esp32s3", ...) pasted into the literal; cppcheck cannot know it.
    return "xtensa-" CONFIG_IDF_TARGET "-elf-";
#endif
  }

protected:
  /// The dedicated core dump data partition (nullptr if the partition table
  /// has none).
  static const esp_partition_t *coredump_partition() {
    return esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
                                    nullptr);
  }
};

} // namespace espp
