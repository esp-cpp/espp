#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"

#include "base_component.hpp"

namespace espp {

/**
 * @brief Transport-agnostic OTA (over-the-air) firmware update engine.
 *
 * Wraps ESP-IDF's `esp_ota_ops` in an idiomatic espp API: no exceptions, all
 * failures reported via `std::error_code`, and a single mutex-serialized
 * update session (`begin()` -> `write()`... -> `finish()` / `abort()`). The
 * component performs no I/O of its own — feed it image bytes from ANY
 * transport (USB vendor / WebUSB stream, HTTP request body, TCP socket, UART,
 * SD card, ...) and it streams them into the next OTA app partition.
 *
 * On the first chunk of `write()` the image is sanity-checked (the ESP image
 * magic byte 0xE9) and the embedded application descriptor
 * (`esp_app_desc_t`: project name, version, build date) is extracted, logged
 * and exposed via `incoming_app_description()`; with
 * `Config::reject_same_version` set, an image whose version matches the
 * running app is rejected. `finish()` runs the full image validation
 * (including the appended SHA-256) via `esp_ota_end()` and sets the boot
 * partition, but does NOT restart — call `restart()` when the application is
 * ready to reboot into the new image.
 *
 * @section ota_rollback Rollback
 *
 * The rollback helpers (`is_pending_verify()`, `mark_app_valid()`,
 * `mark_app_invalid_and_rollback()`) require the bootloader rollback support
 * to be compiled in (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`). With rollback
 * enabled, a freshly-updated app boots in the `ESP_OTA_IMG_PENDING_VERIFY`
 * state and MUST call `mark_app_valid()` after its own health checks pass —
 * otherwise the bootloader rolls back to the previous image on the next
 * reset. Call `mark_app_invalid_and_rollback()` to actively reject the new
 * image and reboot into the previous one.
 *
 * \section ota_ex1 OTA Example (USB / WebUSB + WiFi / Ethernet HTTP)
 * \snippet ota_example.cpp ota_example
 */
class Ota : public BaseComponent {
public:
  /// Application description extracted from an app image's `esp_app_desc_t`.
  struct AppDescription {
    std::string project_name; ///< Project name (CMake project / PROJECT_NAME).
    std::string version;      ///< Application version (git describe / PROJECT_VER).
    std::string date;         ///< Compile date.
    std::string time;         ///< Compile time.
    std::string idf_version;  ///< ESP-IDF version the image was built with.
  };

  /**
   * @brief Progress callback, invoked (with the session mutex held, so keep it
   *        short) after every successful write().
   * @param written Total bytes written to the update partition so far.
   * @param total Total expected image size in bytes (0 if unknown / streaming).
   */
  using progress_callback_fn = std::function<void(size_t written, size_t total)>;

  /// Configuration for the Ota engine.
  struct Config {
    /// Reject an incoming image whose `esp_app_desc_t` version string matches
    /// the running app's version (checked on the first chunk of write()).
    bool reject_same_version{false};
    /// Optional progress callback (see progress_callback_fn).
    progress_callback_fn progress_callback{nullptr};
    /// Logger verbosity.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  /**
   * @brief Construct the OTA engine. Does not touch the flash until begin().
   * @param config Configuration parameters.
   */
  explicit Ota(const Config &config)
      : BaseComponent("Ota", config.log_level)
      , config_(config) {}

  /// Aborts any still-active update session.
  ~Ota() {
    std::error_code ec;
    abort(ec);
  }

  // Non-copyable, non-movable (owns an esp_ota handle + mutex).
  Ota(const Ota &) = delete;
  Ota &operator=(const Ota &) = delete;

  /**
   * @brief Start an update session targeting the next OTA app partition
   *        (esp_ota_get_next_update_partition()).
   * @param image_size Expected image size in bytes, or 0 if unknown /
   *        streaming (OTA_SIZE_UNKNOWN — the WHOLE update partition is erased
   *        up front, which can take several seconds; with a known size only
   *        the required range is erased).
   * @param[out] ec Set on failure: a session is already active
   *        (device_or_resource_busy), no OTA partition exists (no_such_device),
   *        the image is larger than the partition (no_space_on_device), or the
   *        flash erase failed (io_error).
   * @return true if the session started, false otherwise (ec is set).
   */
  bool begin(size_t image_size, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    ec.clear();
    if (session_active_) {
      logger_.error("begin: an update session is already active ({} bytes written)",
                    bytes_written_);
      ec = std::make_error_code(std::errc::device_or_resource_busy);
      return false;
    }
    const esp_partition_t *update = esp_ota_get_next_update_partition(nullptr);
    if (update == nullptr) {
      logger_.error("begin: no OTA update partition found (partition table needs >= 2 ota slots)");
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    if (image_size > update->size) {
      logger_.error("begin: image size {} exceeds update partition '{}' size {}", image_size,
                    update->label, update->size);
      ec = std::make_error_code(std::errc::no_space_on_device);
      return false;
    }
    logger_.info("begin: erasing + opening update partition '{}' ({} bytes) for a {} image",
                 update->label, update->size,
                 image_size ? std::to_string(image_size) + "-byte" : "streaming (unknown size)");
    const esp_err_t err =
        esp_ota_begin(update, image_size == 0 ? OTA_SIZE_UNKNOWN : image_size, &handle_);
    if (err != ESP_OK) {
      logger_.error("begin: esp_ota_begin failed: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    session_active_ = true;
    update_partition_ = update;
    image_size_ = image_size;
    bytes_written_ = 0;
    header_.clear();
    header_.reserve(kAppDescEnd);
    incoming_desc_.reset();
    return true;
  }

  /**
   * @brief Stream image bytes into the update partition.
   *
   * The first chunk(s) are additionally used to validate the ESP image magic
   * byte (0xE9) and — once enough bytes have arrived — to extract and log the
   * incoming `esp_app_desc_t` (see incoming_app_description()). On ANY failure
   * the session is aborted (a partially-written image is useless), so after an
   * error the caller may simply start over with begin().
   *
   * @param data Image bytes (any chunk size; empty is a no-op).
   * @param[out] ec Set on failure: no active session (operation_not_permitted),
   *        bad image magic (illegal_byte_sequence), same-version rejection
   *        (file_exists, when Config::reject_same_version is set), or a flash
   *        write failure (io_error).
   * @return true if the bytes were written, false otherwise (ec is set).
   */
  bool write(std::span<const uint8_t> data, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    ec.clear();
    if (!session_active_) {
      logger_.error("write: no active update session (call begin() first)");
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    if (data.empty())
      return true;
    // Validate the very first image byte: every ESP app image starts with the
    // image header magic 0xE9.
    if (bytes_written_ == 0 && header_.empty() && data[0] != ESP_IMAGE_HEADER_MAGIC) {
      logger_.error("write: first image byte 0x{:02x} != ESP image magic 0x{:02x}; aborting",
                    data[0], ESP_IMAGE_HEADER_MAGIC);
      abort_session_locked();
      ec = std::make_error_code(std::errc::illegal_byte_sequence);
      return false;
    }
    // Accumulate the image prefix until the embedded esp_app_desc_t (which
    // follows the image header + first segment header) is complete, then
    // extract / log / (optionally) version-check it.
    if (header_.size() < kAppDescEnd) {
      const size_t take = std::min(kAppDescEnd - header_.size(), data.size());
      header_.insert(header_.end(), data.begin(), data.begin() + take);
      if (header_.size() == kAppDescEnd && !extract_incoming_description_locked(ec))
        return false;
    }
    const esp_err_t err = esp_ota_write(handle_, data.data(), data.size());
    if (err != ESP_OK) {
      logger_.error("write: esp_ota_write failed after {} bytes: {}", bytes_written_,
                    esp_err_to_name(err));
      abort_session_locked();
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    bytes_written_ += data.size();
    if (config_.progress_callback)
      config_.progress_callback(bytes_written_, image_size_);
    return true;
  }

  /**
   * @brief Finish the update: validate the complete image (esp_ota_end() checks
   *        the image structure and its appended SHA-256, plus the signature when
   *        secure boot is enabled) and set it as the boot partition.
   *
   * Does NOT restart the device — call restart() when ready, so the
   * application controls the timing (e.g. after flushing a reply to the host).
   * The session is over after this call, whether it succeeds or fails.
   *
   * @param[out] ec Set on failure: no active session (operation_not_permitted),
   *        image validation failed (illegal_byte_sequence), or setting the boot
   *        partition failed (io_error).
   * @return true if the new image is validated and set to boot, false otherwise.
   */
  bool finish(std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    ec.clear();
    if (!session_active_) {
      logger_.error("finish: no active update session");
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    const esp_partition_t *target = update_partition_;
    const size_t written = bytes_written_;
    // esp_ota_end() invalidates the handle in all outcomes we can hit here, so
    // the session is over regardless of the result.
    const esp_err_t end_err = esp_ota_end(handle_);
    reset_session_locked();
    if (end_err != ESP_OK) {
      logger_.error("finish: esp_ota_end failed after {} bytes: {}", written,
                    esp_err_to_name(end_err));
      ec = std::make_error_code(end_err == ESP_ERR_OTA_VALIDATE_FAILED
                                    ? std::errc::illegal_byte_sequence
                                    : std::errc::io_error);
      return false;
    }
    const esp_err_t boot_err = esp_ota_set_boot_partition(target);
    if (boot_err != ESP_OK) {
      logger_.error("finish: esp_ota_set_boot_partition('{}') failed: {}", target->label,
                    esp_err_to_name(boot_err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    logger_.info("finish: {} bytes validated; boot partition set to '{}' — call restart() to boot "
                 "the new image",
                 written, target->label);
    return true;
  }

  /**
   * @brief Abort the active update session (esp_ota_abort()) and reset the
   *        session state. Idempotent: succeeds as a no-op if no session is
   *        active.
   * @param[out] ec Set on failure (io_error if esp_ota_abort() fails).
   * @return true on success (or no-op), false otherwise (ec is set).
   */
  bool abort(std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    ec.clear();
    if (!session_active_)
      return true;
    logger_.info("abort: discarding update session after {} bytes", bytes_written_);
    if (!abort_session_locked()) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return true;
  }

  /// @brief Restart the device (esp_restart()); does not return. Call after a
  ///        successful finish() to boot the new image.
  [[noreturn]] void restart() {
    logger_.info("restarting...");
    esp_restart();
  }

  /// @brief Whether the RUNNING app is in the ESP_OTA_IMG_PENDING_VERIFY state,
  ///        i.e. it was just installed by an OTA update and must call
  ///        mark_app_valid() after its health checks, or the bootloader will
  ///        roll back on the next reset.
  /// @note Only meaningful with CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y;
  ///       without it this always returns false.
  bool is_pending_verify() const {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state{};
    if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK)
      return false;
    return state == ESP_OTA_IMG_PENDING_VERIFY;
  }

  /**
   * @brief Mark the running app valid and cancel a pending rollback
   *        (esp_ota_mark_app_valid_cancel_rollback()). An app booted in the
   *        pending-verify state must call this once its own health checks pass.
   * @param[out] ec Set on failure (io_error).
   * @return true on success, false otherwise (ec is set).
   */
  bool mark_app_valid(std::error_code &ec) {
    ec.clear();
    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
      logger_.error("mark_app_valid failed: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    logger_.info("running app marked valid; rollback cancelled");
    return true;
  }

  /**
   * @brief Mark the running app invalid and reboot into the previous image
   *        (esp_ota_mark_app_invalid_rollback_and_reboot()). Does not return on
   *        success.
   * @param[out] ec Set on failure (io_error, e.g. no valid app to roll back to
   *        or rollback support not enabled).
   * @return false (only returns on failure; ec is set).
   */
  bool mark_app_invalid_and_rollback(std::error_code &ec) {
    ec.clear();
    logger_.warn("marking running app invalid and rolling back...");
    const esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
    // only reached on failure
    logger_.error("rollback failed: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  /// @brief Label of the partition the current app is running from ("" if unknown).
  std::string running_partition_label() const {
    const esp_partition_t *p = esp_ota_get_running_partition();
    return p ? p->label : "";
  }

  /// @brief Size in bytes of the partition the current app is running from (0 if unknown).
  size_t running_partition_size() const {
    const esp_partition_t *p = esp_ota_get_running_partition();
    return p ? p->size : 0;
  }

  /// @brief Label of the currently-configured BOOT partition ("" if unknown).
  ///        After a successful finish() this is the just-written partition.
  std::string boot_partition_label() const {
    const esp_partition_t *p = esp_ota_get_boot_partition();
    return p ? p->label : "";
  }

  /// @brief Label of the partition the next update session will (or the active
  ///        one does) target ("" if the partition table has no OTA slot).
  std::string update_partition_label() const {
    const esp_partition_t *p = update_partition_for_info();
    return p ? p->label : "";
  }

  /// @brief Size in bytes of the update target partition (0 if none) — the
  ///        maximum image size an update can carry.
  size_t update_partition_size() const {
    const esp_partition_t *p = update_partition_for_info();
    return p ? p->size : 0;
  }

  /// @brief Description (project name / version / build date) of the RUNNING app.
  AppDescription running_app_description() const {
    return to_app_description(*esp_app_get_description());
  }

  /// @brief Description of the INCOMING image, available once enough of the
  ///        image (the first 288 bytes) has been written in the current /
  ///        latest session; std::nullopt before then or if the image carries no
  ///        valid app descriptor.
  std::optional<AppDescription> incoming_app_description() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return incoming_desc_;
  }

  /// @brief Whether an update session is active (begin() succeeded and neither
  ///        finish() nor abort() has ended it).
  bool session_active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_active_;
  }

  /// @brief Bytes written to the update partition in the current session.
  size_t bytes_written() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bytes_written_;
  }

  /// @brief Expected image size passed to begin() (0 if unknown / streaming).
  size_t image_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return image_size_;
  }

protected:
  /// The esp_app_desc_t is embedded right after the image header + first
  /// segment header, so it is complete once this many image bytes have arrived.
  static constexpr size_t kAppDescOffset =
      sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t); // 32
  static constexpr size_t kAppDescEnd = kAppDescOffset + sizeof(esp_app_desc_t);

  /// Copy a fixed-size (possibly unterminated) char array field to a string.
  template <size_t N> static std::string field_to_string(const char (&field)[N]) {
    return std::string(field, strnlen(field, N));
  }

  static AppDescription to_app_description(const esp_app_desc_t &desc) {
    AppDescription out;
    out.project_name = field_to_string(desc.project_name);
    out.version = field_to_string(desc.version);
    out.date = field_to_string(desc.date);
    out.time = field_to_string(desc.time);
    out.idf_version = field_to_string(desc.idf_ver);
    return out;
  }

  /// Extract + log the incoming image's esp_app_desc_t from the accumulated
  /// header bytes; with Config::reject_same_version set, abort + fail on a
  /// version match. Called with the mutex held.
  bool extract_incoming_description_locked(std::error_code &ec) {
    esp_app_desc_t desc;
    memset(&desc, 0, sizeof(desc));
    memcpy(&desc, header_.data() + kAppDescOffset, sizeof(desc));
    if (desc.magic_word != ESP_APP_DESC_MAGIC_WORD) {
      // Not fatal: the image may still be a valid app image (esp_ota_end()
      // performs the authoritative validation), it just carries no descriptor.
      logger_.warn("incoming image has no valid app descriptor (magic 0x{:08x})", desc.magic_word);
      return true;
    }
    incoming_desc_ = to_app_description(desc);
    logger_.info("incoming firmware: project '{}', version '{}', built {} {} (IDF {})",
                 incoming_desc_->project_name, incoming_desc_->version, incoming_desc_->date,
                 incoming_desc_->time, incoming_desc_->idf_version);
    if (config_.reject_same_version &&
        incoming_desc_->version == field_to_string(esp_app_get_description()->version)) {
      logger_.error("rejecting update: incoming version '{}' matches the running version",
                    incoming_desc_->version);
      abort_session_locked();
      ec = std::make_error_code(std::errc::file_exists);
      return false;
    }
    return true;
  }

  /// esp_ota_abort() + reset the session state. Called with the mutex held.
  bool abort_session_locked() {
    const esp_err_t err = esp_ota_abort(handle_);
    reset_session_locked();
    if (err != ESP_OK) {
      logger_.error("esp_ota_abort failed: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// Reset the session bookkeeping. Called with the mutex held.
  void reset_session_locked() {
    session_active_ = false;
    handle_ = 0;
    update_partition_ = nullptr;
    image_size_ = 0;
    bytes_written_ = 0;
    header_.clear();
    // incoming_desc_ is intentionally kept: it describes the latest image seen.
  }

  /// The partition info getters report the active session's target when a
  /// session is running, else the next update partition.
  const esp_partition_t *update_partition_for_info() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_active_)
      return update_partition_;
    return esp_ota_get_next_update_partition(nullptr);
  }

private:
  Config config_;
  mutable std::mutex mutex_;
  bool session_active_{false};
  esp_ota_handle_t handle_{0};
  const esp_partition_t *update_partition_{nullptr};
  size_t image_size_{0};
  size_t bytes_written_{0};
  std::vector<uint8_t> header_;
  std::optional<AppDescription> incoming_desc_{};
};

} // namespace espp
