#pragma once

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#if defined(ESP_PLATFORM)
#include <dirent.h>
#include <unistd.h>

#include <esp_err.h>
#include <esp_littlefs.h>
#include <esp_partition.h>
#endif // ESP_PLATFORM

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
class FileSystem : public espp::BaseComponent {
public:
#if defined(ESP_PLATFORM) || defined(_DOXYGEN_)
  /// @brief Set whether to mount the file system as read only
  /// @param read_only Whether the file system is mounted as read only
  /// @note This only has an effect if called before the file system is mounted,
  ///       i.e. before the first call to get()
  static void set_mount_as_read_only(bool read_only) { read_only_ = read_only; }

  /// @brief Get whether the file system is mounted as read only
  /// @return Whether the file system is mounted as read only
  static bool is_mount_as_read_only() { return read_only_; }

  /// @brief Set whether to grow the file system on mount
  /// @param grow_on_mount Whether to grow the file system on mount
  /// @note This only has an effect if called before the file system is mounted,
  ///       i.e. before the first call to get()
  static void set_grow_on_mount(bool grow_on_mount) { grow_on_mount_ = grow_on_mount; }

  /// @brief Get whether the file system was grown on mount
  /// @return Whether the file system was grown on mount
  static bool is_grow_on_mount() { return grow_on_mount_; }
#endif // ESP_PLATFORM

  /// @brief Get a human readable string for a byte size
  /// @details
  /// This method returns a human readable string for a byte size.
  /// It is copied from the example on the page:
  /// https://en.cppreference.com/w/cpp/filesystem/file_size
  /// @param bytes The byte size
  /// @return The human readable string
  static std::string human_readable(size_t bytes);

#if defined(ESP_PLATFORM) || defined(_DOXYGEN_)
  /// @brief Get the partition label
  /// @return The partition label
  static const char *get_partition_label() { return CONFIG_ESPP_FILE_SYSTEM_PARTITION_LABEL; }
#endif // ESP_PLATFORM

  /// @brief Get the mount point
  /// @details
  /// The mount point is the root directory of the file system.
  /// It is the root directory of the partition with the partition label.
  /// @see get_root_path()
  /// @return The mount point
  static std::string get_mount_point();

  /// @brief Get the root path
  /// @details
  /// The root path is the root directory of the file system.
  /// @see get_mount_point()
  /// @return The root path
  static std::filesystem::path get_root_path();

  /// @brief Convert file permissions to a string
  /// @details This method converts file permissions to a string in the format "rwxrwxrwx".
  /// @param permissions The file permissions
  /// @return The file permissions as a string
  static std::string to_string(const std::filesystem::perms &permissions);

  /// @brief Convert a time_t to a string
  /// @details This method converts a time_t to a string in the format "Jan 01 00:00".
  /// @param time The time_t to convert
  /// @return The time as a string
  static std::string to_string(time_t time);

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
  size_t get_free_space() const;

  /// @brief Get the total amount of space on the file system
  /// @return The total amount of space in bytes
  size_t get_total_space() const;

  /// @brief Get the amount of used space on the file system
  /// @return The amount of used space in bytes
  size_t get_used_space() const;

  /// @brief Get the time of a file as a string
  /// @details This method gets the time of a file as a string in the format "Jan 01 00:00".
  /// @param path The path to the file
  /// @return The time of the file as a string
  /// @see file_time_to_string()
  std::string get_file_time_as_string(const std::filesystem::path &path) const;

  /// @brief Get a vector of files in a directory
  /// @details This method returns a vector of paths to the files in a directory.
  /// @param path The path to the directory
  /// @param include_directories Whether to include directories in the output
  /// @param recursive Whether to include files in subdirectories
  /// @return A vector of paths to the files in the directory
  std::vector<std::filesystem::path> get_files_in_path(const std::filesystem::path &path,
                                                       bool include_directories = false,
                                                       bool recursive = false);

  /// @brief Completely remove a file or directory (including contents)
  /// @details This method removes a file or directory and all of its contents.
  ///          If the path is a directory, it will iterate over the contents
  ///          and remove them recursively. If the path is a file, it will
  ///          remove the file. If the path does not exist, it will return false.
  /// @param path The path to the file or directory
  /// @param ec The error code to set if an error occurs
  /// @return Whether the file or directory was successfully removed
  bool remove(const std::filesystem::path &path, std::error_code &ec);

  /// @brief Remove the contents of a directory, but not the directory itself
  /// @details This method removes the contents of a directory, but not the
  ///          directory itself. If the path is not a directory, it will return
  ///          false. If the path does not exist, it will return false. If the
  ///          path is a directory, it will iterate over the contents and remove
  ///          them recursively.
  /// @param path The path to the directory
  /// @param ec The error code to set if an error occurs
  /// @return Whether the contents of the directory were successfully removed
  bool remove_contents(const std::filesystem::path &path, std::error_code &ec);

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
                             const std::string &prefix = "");

  /// @brief Get a file entry formatted as a string
  /// @details
  /// This method returns a string formatted as a file entry. The file entry
  /// contains the type, permissions, number of links, owner, group, size,
  /// date and time, and name of the file. The file entry is formatted according
  /// to the config. The config is a struct with boolean values for each of the
  /// fields to include in the output. The fields are:
  /// - type: The type of the file (directory, file, etc.)
  /// - permissions: The permissions of the file
  /// - number_of_links: The number of links to the file
  /// - owner: The owner of the file
  /// - group: The group of the file
  /// - size: The size of the file
  /// - date_time: The date and time of the file
  /// @param path The path to the file
  /// @param config The config for the output
  /// @param prefix The prefix to use for the output
  /// @return The file entry as a string
  /// @see file_entry_string()
  std::string file_entry_string(const std::filesystem::path &path, const ListConfig &config,
                                const std::string &prefix = "");

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
  static bool read_only_;     ///< Whether the file system is read only
  static bool grow_on_mount_; ///< Whether to grow the file system on mount

  /// @brief Constructor
  /// @details
  /// The constructor is private to ensure that the class is a singleton.
  FileSystem();

  /// @brief Initialize the file system
  /// @details
  /// This method initializes the file system. It is protected and called only by the constructor.
  /// It is responsible for mounting the file system and creating the root directory.
  void init();

  /// @brief Remove a file
  /// @param path The path to the file
  /// @return Whether the file was successfully removed
  bool remove_file(const std::filesystem::path &path);

  /// @brief Remove an empty directory
  /// @param path The path to the empty directory
  /// @return Whether the directory was successfully removed
  /// @note This method does not remove the contents of the directory, only the
  ///       directory itself.
  /// @warning If the directory is not empty, it will not be removed, and the
  ///          method will return false.
  bool remove_directory(const std::filesystem::path &path);
};
} // namespace espp
