#pragma once

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/stdio.h>
#include <sys/types.h>

#include <esp_err.h>
#include <esp_partition.h>

#include <esp_littlefs.h>

#include "logger.hpp"

namespace espp {
/// @brief File system class
/// @details
/// This class is a singleton and should be accessed via the get() method.
/// The class is responsible for mounting the file system and providing
/// access to the file system. It is configured via the menuconfig system and will
/// use the partition with the label specified in the menuconfig. The partition
/// must be formatted with the LittleFS file system. The file system is mounted
/// at the root directory of the partition such that all files will be stored
/// under the path "/<partition_label>/".
class FileSystem {
public:
  /// @brief Access the singleton instance of the file system
  /// @return Reference to the file system instance
  static FileSystem &get() {
    static FileSystem instance;
    return instance;
  }

  FileSystem(const FileSystem &) = delete;
  FileSystem &operator=(const FileSystem &) = delete;
  FileSystem(FileSystem &&) = delete;
  FileSystem &operator=(FileSystem &&) = delete;

  /// @brief Get the amount of free space on the file system
  /// @return The amount of free space in bytes
  size_t get_free_space() {
    size_t total, used;
    esp_littlefs_info(get_partition_label(), &total, &used);
    return total - used;
  }

  /// @brief Get the total amount of space on the file system
  /// @return The total amount of space in bytes
  size_t get_total_space() {
    size_t total, used;
    esp_littlefs_info(get_partition_label(), &total, &used);
    return total;
  }

  /// @brief Get the amount of used space on the file system
  /// @return The amount of used space in bytes
  size_t get_used_space() {
    size_t total, used;
    esp_littlefs_info(get_partition_label(), &total, &used);
    return used;
  }

  /// @brief Get a human readable string for a byte size
  /// @details
  /// This method returns a human readable string for a byte size.
  /// It is copied from the example on the page:
  /// https://en.cppreference.com/w/cpp/filesystem/file_size
  /// @param bytes The byte size
  /// @return The human readable string
  std::string human_readable(size_t bytes) {
    int i{};
    double mantissa = bytes;
    for (; mantissa >= 1024.; mantissa /= 1024., ++i) {
    }
    mantissa = std::ceil(mantissa * 10.) / 10.;
    return i == 0 ? fmt::format("{}{}", mantissa, "BKMGTPE"[i]) : fmt::format("B ({})", bytes);
  }

  /// @brief Get the partition label
  /// @return The partition label
  const char *get_partition_label() { return CONFIG_ESPP_FILE_SYSTEM_PARTITION_LABEL; }

  /// @brief Get the mount point
  /// @details
  /// The mount point is the root directory of the file system.
  /// It is the root directory of the partition with the partition label.
  /// @see get_root_path() and get_partition_label()
  /// @return The mount point
  std::string get_mount_point() { return "/" + std::string{get_partition_label()}; }

  /// @brief Get the root path
  /// @details
  /// The root path is the root directory of the file system.
  /// @see get_mount_point() and get_partition_label()
  /// @return The root path
  std::filesystem::path get_root_path() { return std::filesystem::path{get_mount_point()}; }

protected:
  /// @brief Constructor
  /// @details
  /// The constructor is private to ensure that the class is a singleton.
  FileSystem() : logger_({.tag = "FileSystem", .level = Logger::Verbosity::WARN}) { init(); }

  /// @brief Initialize the file system
  /// @details
  /// This method initializes the file system. It is protected and called only by the constructor.
  /// It is responsible for mounting the file system and creating the root directory.
  void init() {
    logger_.debug("Initializing file system");
    esp_err_t err;

    // Initialize LittleFS
    esp_vfs_littlefs_conf_t conf = {
        .base_path = get_mount_point().c_str(),
        .partition_label = get_partition_label(),
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    err = esp_vfs_littlefs_register(&conf);
    switch (err) {
    case ESP_OK:
      logger_.debug("LittleFS initialized");
      break;
    case ESP_ERR_NOT_FOUND:
      logger_.error("Failed to find partition with label %s", get_partition_label());
      break;
    case ESP_FAIL:
      logger_.error("Failed to mount or format filesystem");
      break;
    case ESP_ERR_INVALID_STATE:
      logger_.error("Failed to initialize LittleFS, invalid state");
      break;
    default:
      logger_.error("Failed to initialize LittleFS, unknown error");
      break;
    }
    // NOTE: this will raise an exception if the LittleFS is not initialized
    ESP_ERROR_CHECK(err);

    // Create root directory
    std::filesystem::path root_path = get_root_path();
    if (!std::filesystem::exists(root_path)) {
      logger_.debug("Creating root directory");
      std::filesystem::create_directory(root_path);
    }
  }

  Logger logger_;
};
} // namespace espp
