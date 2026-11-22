#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "gamepad_map.hpp"
#include "persistent_data.hpp"

namespace espp {

/**
 * @brief The map for one type of controller.
 *
 * @details A controller type is its USB / Bluetooth vendor and product ID
 *          (for BLE, from the Device Information service's PnP ID). A
 *          controller that reports neither (VID and PID 0) is matched by its
 *          name instead.
 */
struct GamepadProfile {
  /// Longest name stored; longer names are cut to it (and compared cut).
  static constexpr size_t MAX_NAME_LENGTH = 64;

  uint16_t vid{0};    ///< vendor ID (0 = unknown: match by name)
  uint16_t pid{0};    ///< product ID (0 = unknown: match by name)
  std::string name{}; ///< for display, and the key when VID and PID are 0
  GamepadMap map{};   ///< the mapping for this controller type

  /// @brief A name as stored (cut to MAX_NAME_LENGTH bytes).
  static std::string clip_name(const std::string &n) { return n.substr(0, MAX_NAME_LENGTH); }

  /// @brief Whether this profile is for the controller with these IDs / name.
  bool matches(uint16_t vid_, uint16_t pid_, const std::string &name_) const {
    if (vid == 0 && pid == 0)
      return vid_ == 0 && pid_ == 0 && !name.empty() && clip_name(name) == clip_name(name_);
    return vid == vid_ && pid == pid_;
  }
  /// @brief Whether any controller can match this profile.
  bool matchable() const { return vid != 0 || pid != 0 || !name.empty(); }

  bool operator==(const GamepadProfile &) const = default;
};

/// @brief The data a GamepadProfiles store keeps (schema version 1).
struct GamepadProfilesV1 {
  static constexpr uint32_t VERSION = 1;
  GamepadMap default_map{};               ///< for a controller without a profile
  std::vector<GamepadProfile> profiles{}; ///< per controller type
};

/**
 * @brief Per-controller-type gamepad maps, stored in a file with
 *        PersistentData.
 *
 * @details Holds a default map plus one GamepadProfile per controller type,
 *          and saves every change atomically (see PersistentData) to a file on
 *          a mounted filesystem (LittleFS, SPIFFS, SD). Look a controller up
 *          when it connects (map_for()) and keep the returned map for its
 *          reports; the lookup copies the map and is not meant for every
 *          report.
 *
 *          Every map that comes in is checked with GamepadMap::valid(). From a
 *          file, an invalid rule is dropped (the rest of its map kept), and a
 *          profile no controller can match, a duplicate or one over the limit
 *          is dropped, each with a warning. A file that cannot be read (corrupt,
 *          or written by a newer firmware) is renamed to `<file>.unreadable`
 *          and the store starts from the defaults, so the next change does not
 *          overwrite what may still be recoverable (see load_error()).
 *
 *          Thread-safe.
 *
 * \section gamepad_profiles_ex1 Gamepad Profiles Example
 * \snippet gamepad_map_example.cpp gamepad profiles example
 */
class GamepadProfiles : public BaseComponent {
public:
  using Data = GamepadProfilesV1;
  using Storage = PersistentData<GamepadProfilesV1>;

  /**
   * @brief Configuration for the GamepadProfiles.
   */
  struct Config {
    std::filesystem::path file_path; ///< file the profiles live in
    GamepadMap default_map{};        ///< default map when the file does not exist yet
    size_t max_profiles{16};         ///< profiles the store accepts
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Verbosity of the logger
  };

  /**
   * @brief Load the profiles (or start from `config.default_map` when there is
   *        no file yet; the file is written on the first change).
   * @param config The configuration.
   */
  explicit GamepadProfiles(const Config &config)
      : BaseComponent("GamepadProfiles", config.log_level)
      , max_profiles_(config.max_profiles)
      , storage_({.file_path = config.file_path,
                  .auto_load = false,
                  .max_file_size = max_file_size(config.max_profiles),
                  .log_level = config.log_level}) {
    std::error_code ec;
    if (storage_.load(ec)) {
      data_ = sanitize(storage_.get());
    } else {
      if (ec != make_error_code(PersistentDataErrc::file_not_found)) {
        load_error_ = ec;
        keep_unreadable(config.file_path, ec);
      }
      data_.default_map = config.default_map.valid() ? config.default_map : GamepadMap{};
    }
    // only data_ is used from here on: do not keep a second copy in storage_
    storage_.set(Data{});
  }

  /// @brief Why the file could not be loaded (it was set aside as
  ///        `<file>.unreadable`); clear if it loaded or did not exist.
  std::error_code load_error() const { return load_error_; }

  /// @brief The map for a controller: its profile's, else the default map.
  GamepadMap map_for(uint16_t vid, uint16_t pid, const std::string &name = {}) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto *p = find(vid, pid, name);
    return p ? p->map : data_.default_map;
  }

  /// @brief The profile for a controller, if it has one.
  std::optional<GamepadProfile> profile(uint16_t vid, uint16_t pid,
                                        const std::string &name = {}) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto *p = find(vid, pid, name);
    return p ? std::optional<GamepadProfile>(*p) : std::nullopt;
  }

  /// @brief Whether a controller has a profile of its own.
  bool has_profile(uint16_t vid, uint16_t pid, const std::string &name = {}) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return find(vid, pid, name) != nullptr;
  }

  /// @brief Every profile.
  std::vector<GamepadProfile> profiles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.profiles;
  }

  /// @brief How many profiles are stored.
  size_t profile_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.profiles.size();
  }

  /// @brief The map for controllers without a profile.
  GamepadMap default_map() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.default_map;
  }

  /// @brief Everything, as stored.
  Data data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  /// @brief The most profiles the store accepts.
  size_t max_profiles() const { return max_profiles_; }

  /**
   * @brief Replace the default map and save.
   * @param map The map (must be valid()).
   * @param ec Set on failure: std::errc::invalid_argument for an invalid map,
   *           else the save error.
   * @return true if saved.
   */
  bool set_default_map(const GamepadMap &map, std::error_code &ec) {
    if (!check(map, ec))
      return false;
    std::lock_guard<std::mutex> lock(mutex_);
    Data next = data_;
    next.default_map = map;
    return commit(std::move(next), ec);
  }

  /**
   * @brief Add a profile, or replace the one for the same controller, and save.
   * @param entry The profile (its map must be valid(); with VID and PID 0 it
   *                needs a name; a name is stored cut to MAX_NAME_LENGTH).
   * @param ec Set on failure: std::errc::invalid_argument for an invalid
   *           profile, std::errc::no_buffer_space when the store is full, else
   *           the save error.
   * @return true if saved.
   */
  bool set_profile(const GamepadProfile &entry, std::error_code &ec) {
    if (!check(entry.map, ec))
      return false;
    if (entry.vid == 0 && entry.pid == 0 && entry.name.empty()) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    GamepadProfile stored = entry;
    stored.name = GamepadProfile::clip_name(entry.name);
    std::lock_guard<std::mutex> lock(mutex_);
    Data next = data_;
    auto it =
        std::find_if(next.profiles.begin(), next.profiles.end(), [&](const GamepadProfile &p) {
          return p.matches(stored.vid, stored.pid, stored.name);
        });
    if (it != next.profiles.end()) {
      *it = std::move(stored);
    } else if (next.profiles.size() >= max_profiles_) {
      ec = std::make_error_code(std::errc::no_buffer_space);
      return false;
    } else {
      next.profiles.push_back(std::move(stored));
    }
    return commit(std::move(next), ec);
  }

  /**
   * @brief Remove a controller's profile (it then uses the default map) and save.
   * @return true if saved; false with `ec` clear if there was no such profile.
   */
  bool erase_profile(uint16_t vid, uint16_t pid, const std::string &name, std::error_code &ec) {
    ec.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    Data next = data_;
    const auto before = next.profiles.size();
    std::erase_if(next.profiles,
                  [&](const GamepadProfile &p) { return p.matches(vid, pid, name); });
    if (next.profiles.size() == before)
      return false;
    return commit(std::move(next), ec);
  }

  /**
   * @brief Forget every profile and reset the default map to identity, and save.
   * @return true if saved.
   */
  bool reset(std::error_code &ec) {
    std::lock_guard<std::mutex> lock(mutex_);
    return commit(Data{}, ec);
  }

private:
  // An upper bound on the encoded size of a full store (every map at
  // MAX_RULES, every name at MAX_NAME_LENGTH, with generous per-field
  // overhead), so whatever save() writes, load() accepts.
  static size_t max_file_size(size_t max_profiles) {
    constexpr size_t map_bytes = 32 + GamepadMap::MAX_RULES * 16;
    constexpr size_t profile_bytes = map_bytes + GamepadProfile::MAX_NAME_LENGTH + 32;
    return std::max<size_t>(64 * 1024, 1024 + map_bytes + max_profiles * profile_bytes);
  }

  void keep_unreadable(const std::filesystem::path &file, const std::error_code &why) {
    std::filesystem::path aside = file;
    aside += ".unreadable";
    std::error_code ec;
    std::filesystem::remove(aside, ec);
    std::filesystem::rename(file, aside, ec);
    if (ec)
      logger_.error("Could not load {} ({}) nor set it aside ({}): the next change overwrites it",
                    file.string(), why.message(), ec.message());
    else
      logger_.warn("Could not load {} ({}); kept it as {} and started from the defaults",
                   file.string(), why.message(), aside.string());
  }

  const GamepadProfile *find(uint16_t vid, uint16_t pid, const std::string &name) const {
    auto it = std::find_if(data_.profiles.begin(), data_.profiles.end(),
                           [&](const GamepadProfile &p) { return p.matches(vid, pid, name); });
    return it == data_.profiles.end() ? nullptr : &*it;
  }

  static bool check(const GamepadMap &map, std::error_code &ec) {
    ec.clear();
    if (!map.valid()) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    return true;
  }

  // save first, then take it: memory never holds what the file does not
  bool commit(Data next, std::error_code &ec) {
    ec.clear();
    const bool saved = storage_.save(next, ec);
    storage_.set(Data{}); // the file has it; data_ is the one copy in memory
    if (!saved)
      return false;
    data_ = std::move(next);
    return true;
  }

  // Keep what can be kept of a map from the file: its valid rules.
  GamepadMap sanitize_map(GamepadMap m, const char *what) const {
    if (m.left_deadzone_percent > GamepadMap::MAX_DEADZONE_PERCENT ||
        m.right_deadzone_percent > GamepadMap::MAX_DEADZONE_PERCENT) {
      logger_.warn("{}: dead zone out of range; clamped", what);
      m.left_deadzone_percent = std::min(m.left_deadzone_percent, GamepadMap::MAX_DEADZONE_PERCENT);
      m.right_deadzone_percent =
          std::min(m.right_deadzone_percent, GamepadMap::MAX_DEADZONE_PERCENT);
    }
    const size_t before = m.rules.size();
    std::erase_if(m.rules, [](const GamepadMapRule &r) { return !r.valid(); });
    if (m.rules.size() > GamepadMap::MAX_RULES)
      m.rules.resize(GamepadMap::MAX_RULES);
    if (m.rules.size() != before)
      logger_.warn("{}: dropped {} invalid rule(s)", what, before - m.rules.size());
    return m;
  }

  Data sanitize(Data d) const {
    d.default_map = sanitize_map(std::move(d.default_map), "stored default map");
    std::vector<GamepadProfile> kept;
    for (auto &p : d.profiles) {
      p.name = GamepadProfile::clip_name(p.name);
      if (!p.matchable()) {
        logger_.warn("Dropping a stored profile with no VID, PID or name");
        continue;
      }
      const bool dup = std::any_of(kept.begin(), kept.end(), [&](const GamepadProfile &k) {
        return k.matches(p.vid, p.pid, p.name);
      });
      if (dup || kept.size() >= max_profiles_) {
        logger_.warn("Dropping the stored profile for {:04x}:{:04x} '{}': {}", p.vid, p.pid, p.name,
                     dup ? "duplicate" : "over the profile limit");
        continue;
      }
      p.map = sanitize_map(std::move(p.map), "a stored profile");
      kept.push_back(std::move(p));
    }
    d.profiles = std::move(kept);
    return d;
  }

  size_t max_profiles_;
  Storage storage_;
  mutable std::mutex mutex_;
  Data data_{};
  std::error_code load_error_{};
};

} // namespace espp
