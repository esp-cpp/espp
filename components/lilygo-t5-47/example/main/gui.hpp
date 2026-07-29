#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <string_view>

#include "lilygo-t5-47.hpp"

#include "logger.hpp"
#include "task.hpp"

/// The Gui class encapsulates the LVGL UI for the LilyGo T5 4.7" e-paper
/// example. It follows the espp + LVGL pattern used by the other BSP examples:
///
/// * All LVGL objects are created in init_ui(), split into small member
///   functions - one per logical section of the UI.
/// * The class owns a task that calls lv_task_handler(), and a recursive mutex
///   that guards every LVGL call. Public setters lock that mutex so other tasks
///   (the touch callback, a sensor-polling loop, etc.) can call them safely.
/// * Button events are dispatched through a single static trampoline into
///   member handlers.
///
/// Because e-paper updates are slow and cause a visible flash, the setters cache
/// the last value and only touch LVGL (and therefore the panel) when the value
/// actually changes.
///
/// The UI shows a live RTC clock and a toggle-able stats panel (battery, last
/// touch, IO48- and home-button states, LoRa status), plus six buttons: toggle
/// the frontlight, rotate the display, show/hide the stats panel, transmit a
/// LoRa message, full-refresh (clear ghosting), and power off. The panel update
/// mode follows the stats panel - crisp grayscale (GC16) while it is shown so
/// the small text is legible, fast mono (DU) while hidden for a snappy clock.
/// The layout is a flex column so it re-flows when the display is rotated.
class Gui {
public:
  /// Configuration for the Gui
  struct Config {
    // A reference member is always bound at aggregate init (false positive).
    // cppcheck-suppress uninitMemberVarNoCtor
    espp::LilyGoT547 &board;                                          ///< The board BSP
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity
  };

  /// Construct the Gui: build the UI and start the LVGL update task.
  /// @param config The configuration for the Gui
  explicit Gui(const Config &config)
      : board_(config.board)
      , logger_({.tag = "Gui", .level = config.log_level}) {
    init_ui();
    update_task_.start();
  }

  ~Gui() {
    update_task_.stop();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (auto screen = lv_screen_active()) {
      lv_obj_clean(screen);
    }
  }

  /// Set the clock (HH:MM:SS) text. Thread-safe.
  void set_time(std::string_view text);
  /// Set the date (YYYY-MM-DD Weekday) text. Thread-safe.
  void set_date(std::string_view text);
  /// Set the battery status text. Thread-safe.
  void set_battery(std::string_view text);
  /// Set the last-touch text from a touch event. Thread-safe.
  void set_touch(int num_points, int x, int y);
  /// Set the IO48 button state. Thread-safe.
  void set_io48_button(bool pressed);
  /// Set the home button state. Thread-safe.
  void set_home_button(bool pressed);
  /// Set the LoRa status text (e.g. last TX / RX). Thread-safe.
  void set_lora(std::string_view text);

  /// Consume a pending "LoRa send" request from the LoRa button. The example's
  /// main loop calls this and performs the (blocking) transmit off the UI task.
  /// \return true if a send was requested since the last call
  bool take_lora_send_request() { return lora_send_requested_.exchange(false); }

  /// Do a full-screen grayscale refresh (clears ghosting), serialized with the
  /// LVGL flush so it never races the update task. Thread-safe.
  void request_full_refresh();

protected:
  void init_ui();
  void init_background();
  void init_header();
  void init_info_panel();
  void init_buttons();

  // create a white e-paper-friendly button with a black border and a label
  lv_obj_t *make_button(lv_obj_t *parent, const char *text, lv_obj_t **out_label);

  // update a label only if the text changed (avoids needless e-paper refreshes)
  void set_label_if_changed(lv_obj_t *label, std::string &cache, std::string_view text);

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // single trampoline for all button events
  static void event_callback(lv_event_t *e);
  void on_clicked(lv_event_t *e);

  espp::LilyGoT547 &board_;

  // LVGL objects
  lv_obj_t *time_label_{nullptr};
  lv_obj_t *date_label_{nullptr};
  lv_obj_t *battery_label_{nullptr};
  lv_obj_t *touch_label_{nullptr};
  lv_obj_t *io48_label_{nullptr};
  lv_obj_t *home_label_{nullptr};
  lv_obj_t *lora_label_{nullptr};
  lv_obj_t *frontlight_button_{nullptr};
  lv_obj_t *frontlight_button_label_{nullptr};
  lv_obj_t *rotate_button_{nullptr};
  lv_obj_t *stats_button_{nullptr};
  lv_obj_t *lora_button_{nullptr};
  lv_obj_t *refresh_button_{nullptr};
  lv_obj_t *off_button_{nullptr};

  // cached label strings, to skip unchanged updates
  std::string time_cache_;
  std::string date_cache_;
  std::string battery_cache_;
  std::string touch_cache_;
  std::string io48_cache_;
  std::string home_cache_;
  std::string lora_cache_;

  std::atomic<bool> lora_send_requested_{false}; // set by the LoRa button
  bool frontlight_on_{false};
  // Stats panel visible by default. The update mode follows this: crisp GC16
  // while stats are shown (the small text needs grayscale to be legible), fast
  // mono DU while hidden (a snappy clock-only view).
  bool stats_shown_{true};
  std::atomic<bool> rotate_requested_{false};       // set by the Rotate button, handled in update()
  std::atomic<bool> stats_toggle_requested_{false}; // set by the Stats button, handled in update()

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 8 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
