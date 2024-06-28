#include "file_system.hpp"

using namespace espp;

bool FileSystem::read_only_ = false;
bool FileSystem::grow_on_mount_ = true;

size_t FileSystem::get_free_space() const {
  size_t total, used;
  esp_littlefs_info(get_partition_label(), &total, &used);
  return total - used;
}

size_t FileSystem::get_total_space() const {
  size_t total, used;
  esp_littlefs_info(get_partition_label(), &total, &used);
  return total;
}

size_t FileSystem::get_used_space() const {
  size_t total, used;
  esp_littlefs_info(get_partition_label(), &total, &used);
  return used;
}

std::string FileSystem::human_readable(size_t bytes) const {
  int i{};
  float mantissa = bytes;
  for (; mantissa >= 1024.; mantissa /= 1024., ++i) {
  }
  mantissa = std::ceil(mantissa * 10.) / 10.;
  return i == 0 ? fmt::format("{}", bytes) : fmt::format("{:.3f}{}", mantissa, "BKMGTPE"[i]);
}

std::string FileSystem::to_string(const std::filesystem::perms &perms) const {
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

std::string FileSystem::to_string(time_t time) const {
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
  return files;
}

std::string FileSystem::list_directory(const std::string &path, const ListConfig &config,
                                       const std::string &prefix) {
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
      result += to_string(perms);
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
      result += fmt::format("{:>12} ", get_file_time_as_string(file_path));
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

void FileSystem::init() {
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
}
