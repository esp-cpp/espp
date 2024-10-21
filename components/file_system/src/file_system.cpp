#include "file_system.hpp"

using namespace espp;

bool FileSystem::read_only_ = false;
bool FileSystem::grow_on_mount_ = true;

FileSystem::FileSystem()
    : espp::BaseComponent("FileSystem") {
  init();
}

std::string FileSystem::get_mount_point() {
#if defined(ESP_PLATFORM)
  return "/" + std::string{get_partition_label()};
#else
  // return the current working directory
  return std::filesystem::current_path().string();
#endif
}

std::filesystem::path FileSystem::get_root_path() {
#if defined(ESP_PLATFORM)
  return std::filesystem::path{get_mount_point()};
#else
  // get current working directory
  return std::filesystem::current_path();
#endif
}

size_t FileSystem::get_free_space() const {
#if defined(ESP_PLATFORM)
  size_t total, used;
  esp_littlefs_info(get_partition_label(), &total, &used);
  return total - used;
#else
  // use std::filesystem to get free space
  std::error_code ec;
  auto space = std::filesystem::space(get_root_path(), ec);
  if (ec) {
    return 0;
  }
  return space.free;
#endif
}

size_t FileSystem::get_total_space() const {
#if defined(ESP_PLATFORM)
  size_t total, used;
  esp_littlefs_info(get_partition_label(), &total, &used);
  return total;
#else
  // use std::filesystem to get total space
  std::error_code ec;
  auto space = std::filesystem::space(get_root_path(), ec);
  if (ec) {
    return 0;
  }
  return space.capacity;
#endif
}

size_t FileSystem::get_used_space() const {
#if defined(ESP_PLATFORM)
  size_t total, used;
  esp_littlefs_info(get_partition_label(), &total, &used);
  return used;
#else
  // use std::filesystem to get used space
  std::error_code ec;
  auto space = std::filesystem::space(get_root_path(), ec);
  if (ec) {
    return 0;
  }
  return space.capacity - space.free;
#endif
}

std::string FileSystem::human_readable(size_t bytes) {
  int i{};
  float mantissa = bytes;
  for (; mantissa >= 1024.; mantissa /= 1024., ++i) {
  }
  mantissa = std::ceil(mantissa * 10.) / 10.;
  return i == 0 ? fmt::format("{}", bytes) : fmt::format("{:.3f}{}", mantissa, "BKMGTPE"[i]);
}

std::string FileSystem::to_string(const std::filesystem::perms &perms) {
  namespace fs = std::filesystem;
  std::string result;
  result += (perms & fs::perms::owner_read) != fs::perms::none ? "r" : "-";
  result += (perms & fs::perms::owner_write) != fs::perms::none ? "w" : "-";
  result += (perms & fs::perms::owner_exec) != fs::perms::none ? "x" : "-";
  result += (perms & fs::perms::group_read) != fs::perms::none ? "r" : "-";
  result += (perms & fs::perms::group_write) != fs::perms::none ? "w" : "-";
  result += (perms & fs::perms::group_exec) != fs::perms::none ? "x" : "-";
  result += (perms & fs::perms::others_read) != fs::perms::none ? "r" : "-";
  result += (perms & fs::perms::others_write) != fs::perms::none ? "w" : "-";
  result += (perms & fs::perms::others_exec) != fs::perms::none ? "x" : "-";
  return result;
}

std::string FileSystem::to_string(time_t time) {
  std::tm tm = *std::localtime(&time);
  char buffer[80];
  std::strftime(buffer, sizeof(buffer), "%b %d %H:%M", &tm);
  return buffer;
}

std::string FileSystem::get_file_time_as_string(const std::filesystem::path &path) const {
  std::error_code ec;
  auto ftime = std::filesystem::last_write_time(path, ec);
  if (ec) {
    return "Jan 01 00:00";
  }
  // NOTE: std::chrono::system_clock::to_time_t is not implemented in ESP-IDF
  // auto cftime = std::chrono::system_clock::to_time_t(ftime);
  auto cftime = to_time_t(ftime);
  return to_string(cftime);
}

std::vector<std::filesystem::path> FileSystem::get_files_in_path(const std::filesystem::path &path,
                                                                 bool include_directories,
                                                                 bool recursive) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    logger_.error("Path does not exist: {}", path.string());
    return {};
  }
  namespace fs = std::filesystem;
  auto file_status = fs::status(path, ec);
  if (ec) {
    logger_.error("Failed to get status for file: {}", path.string());
    return {};
  }
  if (!fs::is_directory(file_status)) {
    logger_.error("Path is not a directory: {}", path.string());
    return {};
  }
  std::vector<std::filesystem::path> files;
#if defined(ESP_PLATFORM)
  // NOTE: we cannot use std::filesystem::directory_iterator because it is not implemented in
  // esp-idf
  DIR *dir = opendir(path.c_str());
  if (dir == nullptr) {
    logger_.error("Failed to open directory: {}", path.string());
    return {};
  }
  struct dirent *entry;
  namespace fs = std::filesystem;
  while ((entry = readdir(dir)) != nullptr) {
    // skip the current and parent directories
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    // use the config to determine output
    auto file_path = fs::path{path} / entry->d_name;
    file_status = fs::status(file_path, ec);
    if (ec) {
      logger_.warn("Failed to get status for file: {}", file_path.string());
    }
    if (fs::is_directory(file_status)) {
      if (include_directories) {
        files.push_back(file_path);
      }
      if (recursive) {
        auto sub_files = get_files_in_path(file_path, include_directories, recursive);
        files.insert(files.end(), sub_files.begin(), sub_files.end());
      }
    } else {
      files.push_back(file_path);
    }
  }
  closedir(dir);
#else
  for (const auto &entry : fs::directory_iterator(path)) {
    auto file_path = entry.path();
    file_status = fs::status(file_path, ec);
    if (ec) {
      logger_.warn("Failed to get status for file: {}", file_path.string());
    }
    if (fs::is_directory(file_status)) {
      if (include_directories) {
        files.push_back(file_path);
      }
      if (recursive) {
        auto sub_files = get_files_in_path(file_path, include_directories, recursive);
        files.insert(files.end(), sub_files.begin(), sub_files.end());
      }
    } else {
      files.push_back(file_path);
    }
  }
#endif
  return files;
}

bool FileSystem::remove(const std::filesystem::path &path, std::error_code &ec) {
  logger_.debug("Removing path: {}", path.string());
  namespace fs = std::filesystem;
  auto file_status = fs::status(path, ec);
  if (std::filesystem::is_directory(file_status)) {
    if (!remove_contents(path, ec)) {
      logger_.error("Failed to remove contents of directory: {}", path.string());
      return false;
    }
    return remove_directory(path);
  } else {
    return remove_file(path);
  }
}

bool FileSystem::remove_file(const std::filesystem::path &path) {
  logger_.debug("Removing file: {}", path.string());
#if defined(ESP_PLATFORM)
  return unlink(path.c_str()) == 0;
#else
  return std::filesystem::remove(path);
#endif
}

bool FileSystem::remove_directory(const std::filesystem::path &path) {
  logger_.debug("Removing directory: {}", path.string());
#if defined(ESP_PLATFORM)
  return rmdir(path.c_str()) == 0;
#else
  return std::filesystem::remove(path);
#endif
}

bool FileSystem::remove_contents(const std::filesystem::path &path, std::error_code &ec) {
  logger_.debug("Removing contents of directory: {}", path.string());
  bool include_directories = true;
  bool recursive = false; // don't want recursive since we'll already recurse
  auto files = get_files_in_path(path, include_directories, recursive);
  bool success = true;
  for (const auto &file : files) {
    if (!remove(file, ec)) {
      logger_.error("Failed to remove file: {}", file.string());
      success = false;
    }
  }
  return success;
}

std::string FileSystem::file_entry_string(const std::filesystem::path &file_path,
                                          const ListConfig &config, const std::string &prefix) {
  std::string result = "";
  // use the config to determine output
  std::error_code ec;
  namespace fs = std::filesystem;
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
    result += to_string(perms);
    result += " ";
  }
  if (config.number_of_links) {
    result += std::to_string(fs::hard_link_count(file_path)) + " ";
  }
  if (config.owner) {
    // NOTE std::filesystem has no way to get file owner
    result += "owner ";
  }
  if (config.group) {
    // NOTE std::filesystem has no way to get file group
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
    result += fmt::format("{:>12} ", get_file_time_as_string(file_path));
  }
  std::string relative_name = (prefix.size() ? (prefix + "/") : "") + file_path.filename().string();
  result += relative_name;
  result += "\r\n";

  return result;
}

std::string FileSystem::list_directory(const std::string &path, const ListConfig &config,
                                       const std::string &prefix) {
  std::string result;
  namespace fs = std::filesystem;
#if defined(ESP_PLATFORM)
  DIR *dir = opendir(path.c_str());
  if (dir == nullptr) {
    return result;
  }
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    // skip the current and parent directories
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    std::error_code ec;
    namespace fs = std::filesystem;
    auto file_path = fs::path{path} / entry->d_name;
    auto file_status = fs::status(file_path, ec);
    result += file_entry_string(file_path, config, prefix);
    if (config.recursive && fs::is_directory(file_status)) {
      std::string relative_name = (prefix.size() ? prefix + "/" : "") + entry->d_name;
      result += list_directory(file_path, config, relative_name);
    }
  }
  closedir(dir);
#else
  for (const auto &entry : fs::directory_iterator(path)) {
    auto file_path = entry.path();
    result += file_entry_string(file_path, config, prefix);
    if (config.recursive && fs::is_directory(entry.status())) {
      std::string relative_name =
          (prefix.size() ? prefix + "/" : "") + file_path.filename().string();
      result += list_directory(entry.path().string(), config, relative_name);
    }
  }
#endif
  return result;
}

void FileSystem::init() {
#if defined(ESP_PLATFORM)
  logger_.debug("Initializing file system");
  esp_err_t err;

  // Initialize LittleFS
  esp_vfs_littlefs_conf_t conf = {
      .base_path = get_mount_point().c_str(),
      .partition_label = get_partition_label(),
      .partition = nullptr,           // use partition label
      .format_if_mount_failed = true, // format if mounting fails
      .read_only = read_only_,
      .dont_mount = false, // mount the filesystem after initialization
      .grow_on_mount = grow_on_mount_,
  };
  err = esp_vfs_littlefs_register(&conf);
  switch (err) {
  case ESP_OK:
    logger_.debug("LittleFS initialized");
    break;
  case ESP_ERR_NOT_FOUND:
    logger_.error("Failed to find partition with label '{}'", get_partition_label());
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
#else
  logger_.debug("Not on ESP platform, no need to initialize file system");
#endif
}
