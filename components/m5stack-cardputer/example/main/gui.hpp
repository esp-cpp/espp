#pragma once

#include <mutex>
#include <string>
#include <string_view>

#include "m5stack-cardputer.hpp"

#include "logger.hpp"
#include "task.hpp"

/// The Gui class encapsulates all of the LVGL UI for this example. It follows
/// the recommended pattern for building UIs with espp + LVGL in C++:
///
/// * All LVGL objects are created in init_ui(), which is broken into small
///   member functions - one per logical piece of the UI.
/// * The class owns the task which calls lv_task_handler(), and a recursive
///   mutex which guards every LVGL call. Public methods lock that mutex, so
///   other tasks (the keyboard scanner callback, the main loop, etc.) can
///   safely call them.
///
/// For this example the Gui is a tiny text editor driven by the Cardputer's
/// keyboard:
/// * A text area fills most of the screen; typed characters are appended to
///   it and backspace / enter / the fn-layer arrow keys edit it.
/// * A status bar at the bottom shows the battery voltage and the most recent
///   key / modifier activity.
class Gui {
public:
  /// Alias for the special (fn layer) keys of the Cardputer keyboard
  using SpecialKey = espp::M5StackCardputer::SpecialKey;

  /// Configuration for the Gui
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /// Construct the Gui: builds the UI and starts the LVGL update task.
  /// @param config The configuration for the Gui
  explicit Gui(const Config &config)
      : logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    update_task_.start();
  }

  ~Gui() {
    update_task_.stop();
    deinit_ui();
  }

  /// Add a character to the text area. Handles backspace ('\b') by deleting
  /// the character before the cursor; other characters (including '\n' and
  /// '\t') are inserted at the cursor. Thread-safe.
  /// @param c The character to add
  void add_char(char c);

  /// Handle a special (fn layer) key: arrows move the cursor, delete removes
  /// the character after the cursor, and esc clears the text area.
  /// Thread-safe.
  /// @param key The special key to handle
  void handle_special_key(SpecialKey key);

  /// Set the text of the status bar. Thread-safe.
  /// @param text The text to display
  void set_status_text(std::string_view text);

  /// Set the text of the IMU overlay label (top-right corner; empty and
  /// invisible until first set). Thread-safe.
  /// @param text The text to display
  void set_imu_text(std::string_view text);

protected:
  void init_ui();
  void deinit_ui();

  // the individual pieces of the UI, called from init_ui()
  void init_background();
  void init_textarea();
  void init_status_bar();
  void init_imu_label();

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // LVGL objects
  lv_obj_t *background_{nullptr};
  lv_obj_t *textarea_{nullptr};
  lv_obj_t *status_label_{nullptr};
  lv_obj_t *imu_label_{nullptr};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 6 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
