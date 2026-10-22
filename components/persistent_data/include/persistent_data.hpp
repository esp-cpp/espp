#pragma once

#include <filesystem>
#include <mutex>
#include <system_error>
#include <vector>

#include "base_component.hpp"

#include "detail/persistent_data_codec.hpp"
#include "persistent_data_errc.hpp"

namespace espp {

/**
 * @brief File-backed storage for one data structure with schema versioning and
 *        forward migration.
 *
 * @details Stores a serializable aggregate (see the serialization component)
 *          in a single file on any filesystem reachable through the VFS
 *          (LittleFS, SPIFFS, SD card). Every file carries a small header with
 *          a magic number, the schema version of the data it holds, a portable
 *          fingerprint of the type's layout, the payload length and a CRC32,
 *          so a load distinguishes "no file yet", "corrupt or truncated",
 *          "written by newer firmware", "version number reused for a different
 *          layout" and "old version, migrated" instead of guessing by trying
 *          to parse. The format is identical on every target and on the host,
 *          so a PC tool can read and write a device's file.
 *
 *          Versions are given as a type list, newest first; the first type is
 *          the current one. Each type declares `static constexpr uint32_t
 *          VERSION` (strictly decreasing along the list) and every type except
 *          the last provides `static T migrate(const Older &)` from the type
 *          after it. A file of an older version is decoded as that version and
 *          walked toward the front of the list one step at a time, so each
 *          version only knows how to come from the one before it. A new
 *          version is prepended; dropping support for the oldest version is
 *          removing the last entry: such files then load with
 *          PersistentDataErrc::unknown_version and the application decides
 *          what to do (typically load_or_create()).
 *
 *          Saves are atomic: the data is written to `<file>.tmp`, synced, then
 *          renamed over the file, so a reset mid-save keeps the previous
 *          contents.
 *
 *          All methods are thread-safe; get() returns a copy and update()
 *          runs the caller's function and the save under the same lock. A save
 *          blocks other callers for the duration of the flash write. For
 *          convenience the object converts to a copy of the data (get()) and
 *          can be assigned from it (set()); saving stays an explicit call
 *          because it can fail.
 *
 * @tparam Versions The schema versions, newest first; the first is the current
 *         type. Each must be an aggregate the serialization component can
 *         (de)serialize.
 *
 * \section persistent_data_ex1 Persistent Data Example
 * \snippet persistent_data_example.cpp persistent data example
 * \section persistent_data_ex2 Migration Example
 * \snippet persistent_data_example.cpp persistent data migration example
 */
template <persistent_data::VersionedType... Versions> class PersistentData : public BaseComponent {
  using List = persistent_data::VersionList<Versions...>;

public:
  /// The current (newest) version type: what get(), set() and save() work with.
  using Current = typename List::Current;

  /// Schema version written by save().
  static constexpr uint32_t CURRENT_VERSION = List::current_version;

  /// Bytes of header in front of the payload in every file.
  static constexpr size_t HEADER_SIZE = persistent_data::HEADER_SIZE;

  /**
   * @brief Configuration for the PersistentData.
   */
  struct Config {
    std::filesystem::path file_path; ///< File to load from / save to
    bool auto_load{true};            ///< Call load() in the constructor (its errors are logged, a
                                     ///< missing file at info level)
    bool save_after_migration{true}; ///< After loading an older version, save the migrated data
                                     ///< so the next load is a plain read
    size_t max_file_size{64 * 1024}; ///< Files larger than this are rejected before being read
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Verbosity of the logger
  };

  /**
   * @brief Construct the PersistentData and, if configured, load the file.
   * @param config The configuration.
   */
  explicit PersistentData(const Config &config)
      : BaseComponent("PersistentData", config.log_level)
      , file_path_(config.file_path)
      , save_after_migration_(config.save_after_migration)
      , max_file_size_(config.max_file_size) {
    set_log_tag("PersistentData:" + file_path_.filename().string());
    if (config.auto_load) {
      std::error_code ec;
      load(ec);
    }
  }

  /// @brief The file this object loads from and saves to.
  const std::filesystem::path &file_path() const { return file_path_; }

  /// @brief Whether a file exists at the configured path.
  bool exists() const { return persistent_data::file_exists(file_path_); }

  /// @brief Whether the data in memory came from, or was saved to, the file.
  /// @details False until a load or save succeeds; get() then returns a
  ///          default-constructed Current.
  bool is_loaded() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return loaded_;
  }

  /// @brief Schema version found in the file by the last successful load.
  /// @return The version, or 0 if nothing was loaded (a save does not change it).
  uint32_t loaded_version() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return load_info_.file_version;
  }

  /// @brief Whether the last successful load migrated an older file.
  bool was_migrated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return load_info_.migrated;
  }

  /// @brief Get a copy of the data in memory.
  /// @note Does not read the file; call load() for that.
  Current get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  /// @brief Replace the data in memory.
  /// @note Does not write the file; call save() for that, or use save(data) / update().
  void set(const Current &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
  }

  /// @brief Assign new data, the same as set(): `settings = Settings{...};`.
  /// @note In memory only; save() still writes the file (and reports errors).
  PersistentData &operator=(const Current &data) {
    set(data);
    return *this;
  }

  /// @brief Convert to a copy of the data in memory, the same as get():
  ///        `Settings s = settings;`.
  operator Current() const { return get(); }

  /**
   * @brief Load the file, migrating it to the current version if needed.
   * @details On success the data in memory is replaced. On failure it is left
   *          as it was and ec says why: file_not_found, io_error,
   *          file_too_large, bad_header, bad_checksum, unknown_version or
   *          deserialize_failed.
   * @param[out] ec The error, if any.
   * @return True if the data was loaded.
   */
  bool load(std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    return load_locked(ec);
  }

  /**
   * @brief Load the file, or create it from `defaults` if there is none.
   * @details Calls load(); if that fails with file_not_found, `defaults` is
   *          saved and becomes the data in memory. Any other load error is
   *          returned untouched (a corrupt file is not silently replaced; call
   *          save(defaults) or erase() for that).
   * @param defaults The data to save when no file exists.
   * @param[out] ec The error, if any.
   * @return True if the data in memory is now valid (loaded or created).
   */
  bool load_or_create(const Current &defaults, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (load_locked(ec))
      return true;
    if (ec != PersistentDataErrc::file_not_found)
      return false;
    logger_.info("No file at '{}', creating it with defaults", file_path_.string());
    data_ = defaults;
    return save_locked(ec);
  }

  /**
   * @brief Save the data in memory to the file (atomically).
   * @param[out] ec The error, if any (serialize_failed or io_error).
   * @return True if the file was written.
   */
  bool save(std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    return save_locked(ec);
  }

  /**
   * @brief Replace the data in memory and save it.
   * @param data The new data.
   * @param[out] ec The error, if any.
   * @return True if the file was written.
   */
  bool save(const Current &data, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = data;
    return save_locked(ec);
  }

  /**
   * @brief Modify the data in memory and save it, under one lock.
   * @param fn Callable taking `Current &`; it may return void, or a bool where
   *           false means "do not save" (the in-memory change is kept).
   * @param[out] ec The error, if any.
   * @return True if the file was written (false also when fn declined).
   */
  template <typename Fn> bool update(Fn &&fn, std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if constexpr (std::is_same_v<std::invoke_result_t<Fn, Current &>, bool>) {
      if (!fn(data_)) {
        ec.clear();
        return false;
      }
    } else {
      fn(data_);
    }
    return save_locked(ec);
  }

  /**
   * @brief Remove the file (and a leftover temporary file).
   * @details The data in memory is kept; is_loaded() becomes false.
   * @param[out] ec The error, if any.
   * @return True if no file exists afterwards (also when there was none).
   */
  bool erase(std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!persistent_data::remove_file(file_path_, ec)) {
      logger_.error("Could not remove '{}': {}", file_path_.string(), ec.message());
      return false;
    }
    loaded_ = false;
    load_info_ = {};
    return true;
  }

  /**
   * @brief Encode data in the file format (header + payload) without writing it.
   * @details For storing the same bytes elsewhere (NVS blob, a transport);
   *          decode() reads them back, migrations included.
   */
  static bool encode(const Current &data, std::vector<uint8_t> &out, std::error_code &ec) {
    return persistent_data::encode(data, CURRENT_VERSION, out, ec);
  }

  /**
   * @brief Decode bytes produced by encode() (any supported version) into `out`.
   * @param[in] bytes The encoded bytes.
   * @param[out] out The decoded, migrated data.
   * @param[out] info Version found and whether it was migrated.
   * @param[out] ec The error, if any.
   */
  static bool decode(std::span<const uint8_t> bytes, Current &out, persistent_data::LoadInfo &info,
                     std::error_code &ec) {
    persistent_data::Header header;
    std::span<const uint8_t> payload;
    if (!persistent_data::parse(bytes, header, payload, ec))
      return false;
    if (!List::decode(header, payload, out, ec))
      return false;
    info.file_version = header.version;
    info.migrated = header.version != CURRENT_VERSION;
    return true;
  }

  /// @brief Portable fingerprint of the current type's serialized layout
  ///        (what save() writes into the header; see layout::fingerprint).
  static uint32_t layout_fingerprint() { return persistent_data::layout::fingerprint<Current>(); }

protected:
  bool load_locked(std::error_code &ec) {
    Current loaded{};
    persistent_data::LoadInfo info;
    if (!persistent_data::load_file<List>(file_path_, max_file_size_, loaded, info, ec)) {
      if (ec == PersistentDataErrc::file_not_found) {
        logger_.info("No file at '{}'", file_path_.string());
      } else {
        logger_.error("Could not load '{}': {}", file_path_.string(), ec.message());
      }
      return false;
    }
    data_ = std::move(loaded);
    loaded_ = true;
    load_info_ = info;
    if (info.migrated) {
      logger_.info("Migrated '{}' from version {} to {}", file_path_.string(), info.file_version,
                   CURRENT_VERSION);
      if (save_after_migration_) {
        std::error_code save_ec;
        if (!save_locked(save_ec)) {
          // the data is loaded fine; only the rewrite failed, so report success
          logger_.warn("Could not save migrated data: {}", save_ec.message());
        }
      }
    } else {
      logger_.debug("Loaded '{}' (version {})", file_path_.string(), info.file_version);
    }
    ec.clear();
    return true;
  }

  bool save_locked(std::error_code &ec) {
    if (!persistent_data::save_file<List>(file_path_, data_, ec)) {
      logger_.error("Could not save '{}': {}", file_path_.string(), ec.message());
      return false;
    }
    loaded_ = true;
    logger_.debug("Saved '{}' (version {})", file_path_.string(), CURRENT_VERSION);
    return true;
  }

  std::filesystem::path file_path_;
  bool save_after_migration_{true};
  size_t max_file_size_{64 * 1024};
  Current data_{};
  bool loaded_{false};
  persistent_data::LoadInfo load_info_{};
  mutable std::mutex mutex_;
};

} // namespace espp
