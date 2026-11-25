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
#include "rumble_options.hpp"

namespace espp {

/**
 * @brief The map and rumble options for one type of controller.
 *
 * @details A controller type is its USB / Bluetooth vendor and product ID
 *          (for BLE, from the Device Information service's PnP ID). A
 *          controller that reports neither (VID and PID 0) is matched by its
 *          name instead.
 */
struct GamepadProfile {
  /// Longest name stored; longer names are cut to it (and compared cut).
  static constexpr size_t MAX_NAME_LENGTH = 64;

  uint16_t vid{0};        ///< vendor ID (0 = unknown: match by name)
  uint16_t pid{0};        ///< product ID (0 = unknown: match by name)
  std::string name{};     ///< for display, and the key when VID and PID are 0
  GamepadMap map{};       ///< the mapping for this controller type
  RumbleOptions rumble{}; ///< how this controller type plays a host's rumble

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

/**
 * @brief One profile as schema version 1 stored it.
 *
 * @details FROZEN: its layout is the V1 file format. Never edit it, nor
 *          GamepadMap / GamepadMapRule while V1 is supported (a GamepadMap
 *          layout change must freeze a GamepadMapV1 for V1 and V2 and add a
 *          V3).
 */
struct GamepadProfileV1 {
  uint16_t vid{0};    ///< vendor ID (0 = unknown: match by name)
  uint16_t pid{0};    ///< product ID (0 = unknown: match by name)
  std::string name{}; ///< for display, and the key when VID and PID are 0
  GamepadMap map{};   ///< the mapping for this controller type
};

/// @brief Schema version 1 of the data a GamepadProfiles store keeps (FROZEN;
///        same layout fingerprint as before rumble options were added).
struct GamepadProfilesV1 {
  static constexpr uint32_t VERSION = 1;
  GamepadMap default_map{};                 ///< for a controller without a profile
  std::vector<GamepadProfileV1> profiles{}; ///< per controller type
};

/// @brief The data a GamepadProfiles store keeps (schema version 2: rumble options).
struct GamepadProfilesV2 {
  static constexpr uint32_t VERSION = 2;
  GamepadMap default_map{};               ///< for a controller without a profile
  RumbleOptions default_rumble{};         ///< for a controller without a profile
  std::vector<GamepadProfile> profiles{}; ///< per controller type

  /// @brief V1 -> V2: maps and keys copied; every rumble option neutral
  ///        (default_rumble included).
  static GamepadProfilesV2 migrate(const GamepadProfilesV1 &v1) {
    GamepadProfilesV2 v2{.default_map = v1.default_map, .default_rumble = {}};
    v2.profiles.reserve(v1.profiles.size());
    for (const auto &p : v1.profiles)
      v2.profiles.push_back(
          {.vid = p.vid, .pid = p.pid, .name = p.name, .map = p.map, .rumble = {}});
    return v2;
  }
};

/**
 * @brief Per-controller-type gamepad maps and rumble options, stored in a file
 *        with PersistentData.
 *
 * @details Holds a default map and default RumbleOptions plus one
 *          GamepadProfile (map and rumble options) per controller type, and
 *          saves every change atomically (see PersistentData) to a file on a
 *          mounted filesystem (LittleFS, SPIFFS, SD). Look a controller up
 *          when it connects (map_for(), rumble_for()) and keep what is
 *          returned for its reports / rumble; the lookups copy and are not
 *          meant for every report.
 *
 *          Every map that comes in is checked with GamepadMap::valid(), and
 *          every RumbleOptions with RumbleOptions::valid(), so nothing invalid
 *          is ever saved. From a file, an invalid rule is dropped (the rest of
 *          its map kept), unknown rumble flag bits are cleared and a rumble
 *          strength over 200 % is clamped, and a profile no controller can
 *          match, a duplicate or one over the limit is dropped, each with a
 *          warning. A file that cannot be read (corrupt, or written by a newer
 *          firmware) is renamed to `<file>.unreadable` and the store starts
 *          from the defaults, so the next change does not overwrite what may
 *          still be recoverable (see load_error()).
 *
 *          The file is schema version 2 (rumble options). A version 1 file
 *          (maps only) loads, migrated in memory with neutral rumble options
 *          everywhere (Config::default_rumble does not apply to it), and stays
 *          version 1 on flash until the first change writes version 2. After
 *          that, older firmware cannot read the file: it sets it aside as
 *          `<file>.unreadable` and starts from its defaults.
 *
 *          Thread-safe.
 *
 * \section gamepad_profiles_ex1 Gamepad Profiles Example
 * \snippet gamepad_map_example.cpp gamepad profiles example
 */
class GamepadProfiles : public BaseComponent {
public:
  using Data = GamepadProfilesV2;
  using Storage = PersistentData<GamepadProfilesV2, GamepadProfilesV1>;

  /**
   * @brief Configuration for the GamepadProfiles.
   */
  struct Config {
    std::filesystem::path file_path; ///< file the profiles live in
    GamepadMap default_map{};        ///< default map when the file does not exist yet
    /// Default rumble options when the file does not exist yet (invalid ones
    /// are replaced by neutral options).
    RumbleOptions default_rumble{};
    size_t max_profiles{16};                              ///< profiles the store accepts
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Verbosity of the logger
  };

  /**
   * @brief Load the profiles (or start from `config.default_map` and
   *        `config.default_rumble` when there is no file yet; the file is
   *        written on the first change).
   * @details An older (version 1) file is migrated in memory only: it is
   *          rewritten as the current version on the first change.
   * @param config The configuration.
   */
  explicit GamepadProfiles(const Config &config)
      : BaseComponent("GamepadProfiles", config.log_level)
      , max_profiles_(config.max_profiles)
      , storage_({.file_path = config.file_path,
                  .auto_load = false,
                  .save_after_migration = false,
                  .max_file_size = max_file_size(config.max_profiles),
                  .log_level = config.log_level}) {
    std::error_code ec;
    if (storage_.load(ec)) {
      data_ = sanitize(storage_.get());
      if (storage_.was_migrated())
        logger_.info("Loaded gamepad profiles v{}; rumble options start neutral; the file is "
                     "rewritten as v2 on the first change",
                     storage_.loaded_version());
    } else {
      if (ec != make_error_code(PersistentDataErrc::file_not_found)) {
        load_error_ = ec;
        keep_unreadable(config.file_path, ec);
      }
      data_.default_map = config.default_map.valid() ? config.default_map : GamepadMap{};
      data_.default_rumble =
          config.default_rumble.valid() ? config.default_rumble : RumbleOptions{};
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

  /// @brief The rumble options for a controller: its profile's, else the default
  ///        rumble options. Apply them once, where the host's raw request
  ///        becomes the controller's motor command (see RumbleOptions).
  RumbleOptions rumble_for(uint16_t vid, uint16_t pid, const std::string &name = {}) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto *p = find(vid, pid, name);
    return p ? p->rumble : data_.default_rumble;
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

  /// @brief The rumble options for controllers without a profile.
  RumbleOptions default_rumble() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_.default_rumble;
  }

  /// @brief Everything, as stored.
  Data data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  /// @brief The most profiles the store accepts.
  size_t max_profiles() const { return max_profiles_; }

  /// @brief A mutex for the store's users to serialize "change the store and
  ///        act on the change" (GamepadMapService holds it across a change and
  ///        the notification it causes, so services sharing a store notify in
  ///        order). The store's own methods never take it.
  std::mutex &change_mutex() const { return change_mutex_; }

  /**
   * @brief Replace the default map and save (the default rumble options are kept).
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
   * @brief Replace the default rumble options and save (the default map is kept).
   * @param rumble The options (must be valid()).
   * @param ec Set on failure: std::errc::invalid_argument for invalid options,
   *           else the save error.
   * @return true if saved.
   */
  bool set_default_rumble(const RumbleOptions &rumble, std::error_code &ec) {
    if (!check(rumble, ec))
      return false;
    std::lock_guard<std::mutex> lock(mutex_);
    Data next = data_;
    next.default_rumble = rumble;
    return commit(std::move(next), ec);
  }

  /**
   * @brief Replace the default map and the default rumble options, and save
   *        both in one commit.
   * @param map The map (must be valid()).
   * @param rumble The rumble options (must be valid()).
   * @param ec Set on failure: std::errc::invalid_argument when either is
   *           invalid (nothing is saved), else the save error.
   * @return true if saved.
   */
  bool set_defaults(const GamepadMap &map, const RumbleOptions &rumble, std::error_code &ec) {
    if (!check(map, ec) || !check(rumble, ec))
      return false;
    std::lock_guard<std::mutex> lock(mutex_);
    Data next = data_;
    next.default_map = map;
    next.default_rumble = rumble;
    return commit(std::move(next), ec);
  }

  /**
   * @brief Add a profile, or replace the one for the same controller, and save.
   * @details Replaces the whole profile, rumble included: `{.vid, .pid, .name,
   *          .map}` resets its rumble to neutral; use set_profile_map() to
   *          change only the map.
   * @param entry The profile (its map and rumble options must be valid(); with
   *                VID and PID 0 it needs a name; a name is stored cut to
   *                MAX_NAME_LENGTH).
   * @param ec Set on failure: std::errc::invalid_argument for an invalid
   *           profile, std::errc::no_buffer_space when the store is full, else
   *           the save error.
   * @return true if saved.
   */
  bool set_profile(const GamepadProfile &entry, std::error_code &ec) {
    return upsert(entry, /*map_only=*/false, ec);
  }

  /**
   * @brief Set a controller's map, keeping its rumble options, and save.
   * @details Replaces only the map (and the stored name, as set_profile()
   *          does) of the controller's profile. A controller without a profile
   *          gets a new one with this map and the default rumble options.
   * @param vid The controller's vendor ID.
   * @param pid The controller's product ID.
   * @param name The controller's name (needed with VID and PID 0; stored cut
   *             to MAX_NAME_LENGTH).
   * @param map The map (must be valid()).
   * @param ec Set on failure: std::errc::invalid_argument for an invalid map
   *           or a missing key, std::errc::no_buffer_space when the store is
   *           full, else the save error.
   * @return true if saved.
   */
  bool set_profile_map(uint16_t vid, uint16_t pid, const std::string &name, const GamepadMap &map,
                       std::error_code &ec) {
    return upsert({.vid = vid, .pid = pid, .name = name, .map = map}, /*map_only=*/true, ec);
  }

  /**
   * @brief Remove a controller's profile (it then uses the default map and the
   *        default rumble options) and save.
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
   * @brief Forget every profile, reset the default map to identity and the
   *        default rumble options to neutral, and save.
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
    constexpr size_t rumble_bytes = 8;
    constexpr size_t profile_bytes =
        map_bytes + rumble_bytes + GamepadProfile::MAX_NAME_LENGTH + 32;
    return std::max<size_t>(64 * 1024,
                            1024 + map_bytes + rumble_bytes + max_profiles * profile_bytes);
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

  static bool check(const RumbleOptions &rumble, std::error_code &ec) {
    ec.clear();
    if (!rumble.valid()) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    return true;
  }

  // set_profile() / set_profile_map(): add the profile, or replace the one for
  // the same controller. With `map_only`, entry.rumble is not used: a stored
  // profile keeps its rumble options and a new one starts from default_rumble.
  bool upsert(GamepadProfile entry, bool map_only, std::error_code &ec) {
    if (!check(entry.map, ec))
      return false;
    if (!map_only && !check(entry.rumble, ec))
      return false;
    if (entry.vid == 0 && entry.pid == 0 && entry.name.empty()) {
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    entry.name = GamepadProfile::clip_name(entry.name);
    std::lock_guard<std::mutex> lock(mutex_);
    Data next = data_;
    auto it =
        std::find_if(next.profiles.begin(), next.profiles.end(), [&](const GamepadProfile &p) {
          return p.matches(entry.vid, entry.pid, entry.name);
        });
    if (it != next.profiles.end()) {
      if (map_only)
        entry.rumble = it->rumble;
      *it = std::move(entry);
    } else if (next.profiles.size() >= max_profiles_) {
      ec = std::make_error_code(std::errc::no_buffer_space);
      return false;
    } else {
      if (map_only)
        entry.rumble = next.default_rumble;
      next.profiles.push_back(std::move(entry));
    }
    return commit(std::move(next), ec);
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

  // Keep what can be kept of rumble options from the file: flag bits this
  // firmware does not know are cleared, a strength over the maximum clamped.
  RumbleOptions sanitize_rumble(RumbleOptions r, const char *what) const {
    const auto unknown = static_cast<unsigned>(r.flags & ~RumbleOptions::KNOWN_FLAGS);
    if (unknown) {
      logger_.warn("{}: unknown rumble flags 0x{:02x} cleared", what, unknown);
      r.flags = static_cast<uint8_t>(r.flags & RumbleOptions::KNOWN_FLAGS);
    }
    if (r.strength_percent > RumbleOptions::MAX_STRENGTH_PERCENT) {
      logger_.warn("{}: rumble strength out of range; clamped", what);
      r.strength_percent = RumbleOptions::MAX_STRENGTH_PERCENT;
    }
    return r;
  }

  Data sanitize(Data d) const {
    d.default_map = sanitize_map(std::move(d.default_map), "stored default map");
    d.default_rumble = sanitize_rumble(d.default_rumble, "stored default rumble");
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
      p.rumble = sanitize_rumble(p.rumble, "a stored profile");
      kept.push_back(std::move(p));
    }
    d.profiles = std::move(kept);
    return d;
  }

  size_t max_profiles_;
  Storage storage_;
  mutable std::mutex mutex_;
  mutable std::mutex change_mutex_;
  Data data_{};
  std::error_code load_error_{};
};

} // namespace espp
