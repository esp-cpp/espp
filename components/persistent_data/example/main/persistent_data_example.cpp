#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "file_system.hpp"
#include "format.hpp"
#include "logger.hpp"
#include "persistent_data.hpp"

using namespace std::chrono_literals;

// ----------------------------------------------------------------------------
// The data we persist, as it evolved over three firmware releases. Every
// version is a plain aggregate (the serialization component needs that) with
// a VERSION constant, and each version after the first knows how to migrate
// from the one before it.
// ----------------------------------------------------------------------------

// Version 1: what the first firmware stored.
struct SettingsV1 {
  static constexpr uint32_t VERSION = 1;
  uint8_t brightness{50};
  std::string device_name{"espp"};
};

// Version 2 added the colour and a list of favourites.
struct SettingsV2 {
  static constexpr uint32_t VERSION = 2;
  uint8_t brightness{50};
  std::string device_name{"espp"};
  uint32_t color_rgb{0x00FF00};
  std::vector<std::string> favorites{};

  static SettingsV2 migrate(const SettingsV1 &v1) {
    return {.brightness = v1.brightness, .device_name = v1.device_name};
  }
};

// Version 3 (current) replaced the 8-bit brightness with a float, nested the
// display settings, and added a boot counter.
struct Settings {
  static constexpr uint32_t VERSION = 3;
  struct Display {
    float brightness{0.5f}; // 0..1
    uint32_t color_rgb{0x00FF00};
  };
  std::string device_name{"espp"};
  Display display{};
  std::vector<std::string> favorites{};
  uint32_t boot_count{0};

  static Settings migrate(const SettingsV2 &v2) {
    return {.device_name = v2.device_name,
            .display = {.brightness = v2.brightness / 100.0f, .color_rgb = v2.color_rgb},
            .favorites = v2.favorites};
  }
};

// A hypothetical version 4 that this firmware does not know about, used by the
// validation section to write a "newer firmware" file.
struct SettingsV4 {
  static constexpr uint32_t VERSION = 4;
  std::string device_name{"future"};
};

template <> struct fmt::formatter<Settings> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }
  template <typename FormatContext> auto format(const Settings &s, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "Settings{{name: '{}', brightness: {:.2f}, color: {:#08x}, favorites: "
                          "{}, boot_count: {}}}",
                          s.device_name, s.display.brightness, s.display.color_rgb, s.favorites,
                          s.boot_count);
  }
};

extern "C" void app_main(void) {
  static espp::Logger logger(
      {.tag = "persistent data example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting persistent data example!");

  // the file system must be mounted first (partition label from menuconfig)
  auto &fs = espp::FileSystem::get();
  std::filesystem::path settings_path = fs.get_root_path() / "settings.bin";

  int passed = 0;
  int failed = 0;
  auto check = [&](const char *name, bool ok) {
    fmt::print("  [{}] {}\n", ok ? "PASS" : "FAIL", name);
    (ok ? passed : failed)++;
  };

  {
    logger.info("Day-to-day use: load or create, then update");
    //! [persistent data example]
    // The type list is oldest first; the last type is the current one. A file
    // written by any listed version loads and is migrated to Settings.
    using PersistentSettings = espp::PersistentData<SettingsV1, SettingsV2, Settings>;
    std::error_code ec;
    PersistentSettings settings({
        .file_path = settings_path,
        .auto_load = false, // we want to handle "no file yet" ourselves below
        .log_level = espp::Logger::Verbosity::INFO,
    });

    // load the file, or create it from defaults if there is none; any other
    // error (corrupt file, newer firmware's file) is reported, not papered over
    if (!settings.load_or_create(Settings{}, ec)) {
      logger.error("Could not load or create settings: {}", ec.message());
    }
    logger.info("Loaded (file version {}): {}", settings.loaded_version(), settings.get());

    // read-modify-write under one lock, saved atomically
    settings.update([](Settings &s) { s.boot_count++; }, ec);
    if (ec) {
      logger.error("Could not save settings: {}", ec.message());
    }

    // get() returns a copy; set() only changes memory, save() writes it
    auto copy = settings.get();
    copy.favorites = {"red", "green"};
    settings.set(copy);
    settings.save(ec);
    //! [persistent data example]
    logger.info("After update: {}", settings.get());
  }

  {
    logger.info("Validating persistent data operations...");
    std::error_code ec;
    using PersistentSettings = espp::PersistentData<SettingsV1, SettingsV2, Settings>;

    // start clean
    {
      PersistentSettings settings({.file_path = settings_path, .auto_load = false});
      check("erase succeeds", settings.erase(ec) && !ec);
      check("file does not exist after erase", !settings.exists());
      check("load of a missing file reports file_not_found",
            !settings.load(ec) && ec == espp::PersistentDataErrc::file_not_found);
      check("is_loaded() is false before any load or save", !settings.is_loaded());
    }

    // create + reload
    {
      PersistentSettings settings({.file_path = settings_path, .auto_load = false});
      Settings defaults{.device_name = "unit", .favorites = {"a"}, .boot_count = 1};
      check("load_or_create creates the file", settings.load_or_create(defaults, ec) && !ec);
      check("file exists after create", settings.exists());
      check("is_loaded() is true after create", settings.is_loaded());
      PersistentSettings again(
          {.file_path = settings_path, .log_level = espp::Logger::Verbosity::WARN});
      check("auto_load loads the created file", again.is_loaded());
      check("loaded data matches", again.get().device_name == "unit" &&
                                       again.get().favorites == std::vector<std::string>{"a"} &&
                                       again.get().boot_count == 1);
      check("loaded_version() is the current version",
            again.loaded_version() == PersistentSettings::CURRENT_VERSION);
      check("was_migrated() is false for a current-version file", !again.was_migrated());
    }

    // update() round trip
    {
      PersistentSettings settings({.file_path = settings_path});
      bool saved = settings.update(
          [](Settings &s) {
            s.boot_count += 10;
            s.display.brightness = 0.75f;
          },
          ec);
      check("update() saves", saved && !ec);
      PersistentSettings again({.file_path = settings_path});
      check("update() result persisted",
            again.get().boot_count == 11 && again.get().display.brightness == 0.75f);
      bool declined = settings.update([](Settings &) { return false; }, ec);
      check("update() returning false does not save", !declined && !ec);
    }

    //! [persistent data migration example]
    // Write a version-1 file the way the first firmware would have, then load
    // it with the full version list: it comes back migrated v1 -> v2 -> v3 and
    // (save_after_migration, the default) the file is rewritten as version 3.
    {
      espp::PersistentData<SettingsV1> old_firmware(
          {.file_path = settings_path, .auto_load = false});
      old_firmware.save(SettingsV1{.brightness = 80, .device_name = "legacy"}, ec);
      check("v1 file written", !ec);

      PersistentSettings settings({.file_path = settings_path});
      check("v1 file loads", settings.is_loaded() && settings.loaded_version() == 1);
      check("v1 file was migrated", settings.was_migrated());
      auto s = settings.get();
      check("migration kept the name", s.device_name == "legacy");
      check("migration converted brightness 80 -> 0.80", s.display.brightness == 0.80f);
      check("migration filled v2 defaults", s.display.color_rgb == 0x00FF00 && s.favorites.empty());
      check("migration filled v3 defaults", s.boot_count == 0);

      PersistentSettings again({.file_path = settings_path});
      check("file was rewritten as the current version",
            again.loaded_version() == PersistentSettings::CURRENT_VERSION && !again.was_migrated());
    }
    //! [persistent data migration example]

    // a file from newer firmware is refused, memory untouched
    {
      espp::PersistentData<SettingsV4> future({.file_path = settings_path, .auto_load = false});
      future.save(SettingsV4{}, ec);
      PersistentSettings settings({.file_path = settings_path, .auto_load = false});
      settings.set(Settings{.device_name = "memory"});
      check("newer file reports unknown_version",
            !settings.load(ec) && ec == espp::PersistentDataErrc::unknown_version);
      check("memory untouched after a failed load", settings.get().device_name == "memory");
      check("load_or_create does not replace a newer file",
            !settings.load_or_create(Settings{}, ec) &&
                ec == espp::PersistentDataErrc::unknown_version);
    }

    // a corrupt file is detected
    {
      std::vector<uint8_t> bytes;
      check("read the raw file", espp::persistent_data::read_file(settings_path, 4096, bytes, ec));
      bytes[PersistentSettings::HEADER_SIZE] ^= 0xFF;
      check("write the corrupted file",
            espp::persistent_data::write_file_atomic(settings_path, bytes, ec));
      PersistentSettings settings({.file_path = settings_path, .auto_load = false});
      check("corrupt file reports bad_checksum",
            !settings.load(ec) && ec == espp::PersistentDataErrc::bad_checksum);
      bytes.resize(PersistentSettings::HEADER_SIZE + 3);
      espp::persistent_data::write_file_atomic(settings_path, bytes, ec);
      check("truncated file reports bad_header",
            !settings.load(ec) && ec == espp::PersistentDataErrc::bad_header);
    }

    // encode / decode without the filesystem (for NVS blobs, transports)
    {
      std::vector<uint8_t> bytes;
      Settings s{.device_name = "bytes", .boot_count = 5};
      check("encode", PersistentSettings::encode(s, bytes, ec) && !ec);
      Settings out{};
      espp::persistent_data::LoadInfo info;
      check("decode", PersistentSettings::decode(bytes, out, info, ec) && !ec);
      check("decode round trip", out.device_name == "bytes" && out.boot_count == 5 &&
                                     info.file_version == 3 && !info.migrated);
    }

    // leave the file in a sane state for the next boot
    {
      PersistentSettings settings({.file_path = settings_path, .auto_load = false});
      settings.save(Settings{.device_name = "espp", .boot_count = 1}, ec);
      check("final save", !ec);
    }

    fmt::print("Persistent data validation: {}/{} checks passed - {}\n", passed, passed + failed,
               failed == 0 ? "PASS" : "FAIL");
  }

  logger.info("Persistent data example complete!");
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
