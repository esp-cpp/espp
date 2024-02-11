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

#include "base_component.hpp"

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
/// @see get_partition_label()
///
/// The class provides methods to get the amount of free, used and total space
/// on the file system. It also provides a method to get a human readable string
/// for a byte size.
///
/// \section fs_ex1 File System Info Example
/// \snippet file_system_example.cpp file_system info example
/// \section fs_ex2 File System POSIX / NEWLIB Example
/// \snippet file_system_example.cpp file_system posix example
/// \section fs_ex3 File System Info std::filesystem Example
/// \snippet file_system_example.cpp file_system std filesystem example
class FileSystem : public BaseComponent {
public:
  /// @brief Access the singleton instance of the file system
  /// @return Reference to the file system instance
  static FileSystem &get() {
    static FileSystem instance;
    return instance;
  }

  /// @brief Config for listing the contents of a directory
  /// @details
  /// This struct is used to configure the output of the list_directory() method.
  /// It contains boolean values for each of the fields to include in the output.
  struct ListConfig {
    bool type = true;            ///< The type of the file (directory, file, etc.)
    bool permissions = true;     ///< The permissions of the file
    bool number_of_links = true; ///< The number of links to the file
    bool owner = true;           ///< The owner of the file
    bool group = true;           ///< The group of the file
    bool size = true;            ///< The size of the file
    bool date_time = true;       ///< The date and time of the file
    bool recursive = false;      ///< Whether to list the contents of subdirectories
  };

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
    float mantissa = bytes;
    for (; mantissa >= 1024.; mantissa /= 1024., ++i) {
    }
    mantissa = std::ceil(mantissa * 10.) / 10.;
    return i == 0 ? fmt::format("{}", bytes) : fmt::format("{:.3f}{}", mantissa, "BKMGTPE"[i]);
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

  /// @brief List the contents of a directory
  /// @details
  /// This method lists the contents of a directory. It returns a string
  /// containing the contents of the directory. The contents are formatted
  /// according to the config. The config is a struct with boolean values
  /// for each of the fields to include in the output. The fields are:
  /// - type: The type of the file (directory, file, etc.)
  /// - permissions: The permissions of the file
  /// - number_of_links: The number of links to the file
  /// - owner: The owner of the file
  /// - group: The group of the file
  /// - size: The size of the file
  /// - date_time: The date and time of the file
  /// - recursive: Whether to list the contents of subdirectories
  /// @param path The path to the directory
  /// @param config The config for the output
  /// @param prefix The prefix to use for the output
  /// @return The contents of the directory
  std::string list_directory(const std::filesystem::path &path, const ListConfig &config,
                             const std::string &prefix = "") {
    return list_directory(path.string(), config, prefix);
  }

  /// @brief List the contents of a directory
  /// @details
  /// This method lists the contents of a directory. It returns a string
  /// containing the contents of the directory. The contents are formatted
  /// according to the config. The config is a struct with boolean values
  /// for each of the fields to include in the output. The fields are:
  /// - type: The type of the file (directory, file, etc.)
  /// - permissions: The permissions of the file
  /// - number_of_links: The number of links to the file
  /// - owner: The owner of the file
  /// - group: The group of the file
  /// - size: The size of the file
  /// - date_time: The date and time of the file
  /// - recursive: Whether to list the contents of subdirectories
  /// @param path The path to the directory
  /// @param config The config for the output
  /// @param prefix The prefix to use for the output
  /// @return The contents of the directory
  std::string list_directory(const std::string &path, const ListConfig &config,
                             const std::string &prefix = "") {
    std::string result;
    DIR *dir = opendir(path.c_str());
    if (dir == nullptr) {
      return result;
    }
    struct dirent *entry;
    namespace fs = std::filesystem;
    while ((entry = readdir(dir)) != nullptr) {
      // skip the current and parent directories
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
        continue;
      }
      // use the config to determine output
      std::error_code ec;
      auto file_path = fs::path{path} / entry->d_name;
      auto file_status = fs::status(file_path, ec);
      if (ec) {
        logger_.warn("Failed to get status for file: {}", file_path.string());
      }
      if (config.type) {
        if (fs::is_directory(file_status)) {
          result += "d";
        } else if (fs::is_regular_file(file_status)) {
          result += "-";
        } else {
          result += "?";
        }
      }
      if (config.permissions) {
        auto perms = file_status.permissions();
        result += (perms & fs::perms::owner_read) != fs::perms::none ? "r" : "-";
        result += (perms & fs::perms::owner_write) != fs::perms::none ? "w" : "-";
        result += (perms & fs::perms::owner_exec) != fs::perms::none ? "x" : "-";
        result += (perms & fs::perms::group_read) != fs::perms::none ? "r" : "-";
        result += (perms & fs::perms::group_write) != fs::perms::none ? "w" : "-";
        result += (perms & fs::perms::group_exec) != fs::perms::none ? "x" : "-";
        result += (perms & fs::perms::others_read) != fs::perms::none ? "r" : "-";
        result += (perms & fs::perms::others_write) != fs::perms::none ? "w" : "-";
        result += (perms & fs::perms::others_exec) != fs::perms::none ? "x" : "-";
        result += " ";
      }
      if (config.number_of_links) {
        result += "1 ";
      }
      if (config.owner) {
        result += "owner ";
      }
      if (config.group) {
        result += "group ";
      }
      if (config.size) {
        if (fs::is_regular_file(file_status)) {
          result += fmt::format("{:>8} ", human_readable(fs::file_size(file_path, ec)));
        } else {
          result += fmt::format("{:>8} ", "");
        }
      }
      if (config.date_time) {
        auto ftime = fs::last_write_time(file_path, ec);
        if (ec) {
          result += "Jan 01 00:00 ";
        } else {
          // NOTE: std::chrono::system_clock::to_time_t is not implemented in ESP-IDF
          // auto cftime = std::chrono::system_clock::to_time_t(ftime);
          auto cftime = to_time_t(ftime);
          std::tm tm = *std::localtime(&cftime);
          char buffer[80];
          std::strftime(buffer, sizeof(buffer), "%b %d %H:%M", &tm);
          result += fmt::format("{:>12} ", buffer);
        }
      }
      result += prefix;
      result += entry->d_name;
      result += "\r\n";
      if (config.recursive && fs::is_directory(file_status)) {
        result += list_directory(file_path, config, std::string{entry->d_name} + "/");
      }
    }
    closedir(dir);
    return result;
  }

  /// Function to convert a time_point to a time_t.
  /// \details This function converts a time_point to a time_t. This function
  ///     is needed because the standard library does not provide a function to
  ///     convert a time_point to a time_t (until c++20 but support seems lacking
  ///     on esp32). This function is taken from
  ///     https://stackoverflow.com/a/61067330
  /// \tparam TP The type of the time_point.
  /// \param tp The time_point to convert.
  /// \return The time_t.
  template <typename TP> static std::time_t to_time_t(TP tp) {
    using namespace std::chrono;
    auto sctp =
        time_point_cast<system_clock::duration>(tp - TP::clock::now() + system_clock::now());
    return system_clock::to_time_t(sctp);
  }

protected:
  /// @brief Constructor
  /// @details
  /// The constructor is private to ensure that the class is a singleton.
  FileSystem()
      : BaseComponent("FileSystem") {
    init();
  }

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
};
} // namespace espp
