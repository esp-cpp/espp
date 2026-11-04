#pragma once

#include <mutex>
#include <string>
#include <string_view>

#include "qemu.hpp"

#include "logger.hpp"
#include "task.hpp"

/// The Gui class encapsulates all of the LVGL UI for this example, following
/// the pattern of the other espp BSP examples: every LVGL object is created in
/// init_ui(), the class owns the task that pumps lv_task_handler() and a
/// recursive mutex that guards every LVGL call, and the public methods lock
/// that mutex so other tasks (the Ethernet event callbacks, the main loop) can
/// update the screen safely.
///
/// The screen shows three status lines (uptime, Ethernet, SD card) and an arc
/// that sweeps continuously, so that a QEMU screenshot or window shows at a
/// glance that LVGL is rendering and the flush path works.
class Gui {
public:
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

  /// Set the uptime line. Thread-safe.
  /// @param text The text to display
  void set_uptime_text(std::string_view text);

  /// Set the Ethernet status line. Thread-safe.
  /// @param text The text to display
  void set_ethernet_text(std::string_view text);

  /// Set the SD card status line. Thread-safe.
  /// @param text The text to display
  void set_sdcard_text(std::string_view text);

protected:
  void init_ui();
  void deinit_ui();

  // the individual pieces of the UI, called from init_ui()
  void init_title();
  void init_status_lines();
  void init_arc();

  // the LVGL update task: advances the arc and calls lv_task_handler()
  // under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  lv_obj_t *title_{nullptr};
  lv_obj_t *uptime_label_{nullptr};
  lv_obj_t *ethernet_label_{nullptr};
  lv_obj_t *sdcard_label_{nullptr};
  lv_obj_t *arc_{nullptr};
  int arc_value_{0};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 8 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
