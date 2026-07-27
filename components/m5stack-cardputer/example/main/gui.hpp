#pragma once

#include <atomic>
#include <deque>
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
/// The UI is organized as a tabview so each subsystem gets its own uncrowded
/// page; the Cardputer has no touchscreen, so tabs are switched from the
/// keyboard (fn+Tab cycles; fn+1 / fn+2 / fn+9 jump to Help / IMU / LoRa). A
/// slim status bar along the bottom stays visible on every tab for transient
/// messages (key names, volume changes, "Sending...").
///
/// * "Text" tab: a text area filling the page; typed characters are appended
///   and backspace / enter / the fn-layer arrow keys edit it. This is also
///   where LoRa messages are composed (fn+0 sends the text area's contents).
/// * "LoRa" tab: the radio status and a scrolling log of sent / received
///   messages with their RSSI / SNR.
/// * "IMU" tab: the live accelerometer / gyroscope readings (ADV only).
/// * "GPS" tab: the GNSS fix status (position, satellites, time) from the
///   LoRa+GPS Cap.
/// * "Sys" tab: board info - variant, battery, and audio volumes.
/// * "Help" tab: the list of controls.
class Gui {
public:
  /// Alias for the special (fn layer) keys of the Cardputer keyboard
  using SpecialKey = espp::M5StackCardputer::SpecialKey;

  /// The tabs of the UI, in bar order. COUNT is the number of tabs.
  enum class Tab : uint8_t {
    TEXT = 0, ///< The text editor / LoRa message composer
    LORA,     ///< LoRa radio status and message log
    IMU,      ///< Live IMU readings (ADV only)
    GPS,      ///< GNSS fix status (LoRa+GPS Cap)
    SYS,      ///< Board / battery / volume info
    HELP,     ///< The controls list
    COUNT,    ///< The number of tabs
  };

  /// The controls for this example; shown on the Help tab and printed to the
  /// log at startup.
  static constexpr const char *HELP_TEXT = "fn+Tab       switch tab\n"
                                           "fn+1/2/9     Help / IMU / LoRa tab\n"
                                           "fn+; / .     scroll (this + other\n"
                                           "             tabs; cursor on Text)\n"
                                           "Type keys to enter text\n"
                                           "fn+, / /     move cursor (Text)\n"
                                           "fn+`         clear text\n"
                                           "fn+bksp      delete forward\n"
                                           "fn+0 (F10)   send text over LoRa\n"
                                           "fn+3 (F3)    record / stop\n"
                                           "fn+4 (F4)    play recording\n"
                                           "fn+5/6       speaker vol -/+\n"
                                           "fn+7/8       mic vol -/+\n"
                                           "G0 button    cycle LED color";

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

  /// Switch to the next tab (wrapping around). Thread-safe.
  /// @return The tab now active
  Tab next_tab();

  /// Select a specific tab. Thread-safe.
  /// @param tab The tab to switch to
  void select_tab(Tab tab);

  /// Get the currently active tab. Thread-safe.
  /// @return The active tab
  Tab active_tab();

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

  /// Get the current contents of the text area (e.g. to send over LoRa).
  /// Thread-safe.
  /// @return The text currently in the text area
  std::string get_text();

  /// Clear the text area. Thread-safe.
  void clear_text();

  /// Set the text of the status bar (transient one-line messages, visible on
  /// every tab). Thread-safe.
  /// @param text The text to display
  void set_status_text(std::string_view text);

  /// Set the text of the IMU tab. Thread-safe.
  /// @param text The text to display
  void set_imu_text(std::string_view text);

  /// Set the text of the Sys (board info) tab. Thread-safe.
  /// @param text The text to display
  void set_system_text(std::string_view text);

  /// Set the text of the GPS tab. Thread-safe.
  /// @param text The text to display
  void set_gps_text(std::string_view text);

  /// Set the LoRa status line (radio state / frequency, or an error), shown
  /// at the top of the LoRa tab. Thread-safe.
  /// @param text The text to display
  void set_lora_status(std::string_view text);

  /// Append a message to the LoRa log (newest at the top; oldest dropped once
  /// full). Thread-safe.
  /// @param text The message line to add (e.g. a received or sent packet)
  void add_lora_message(std::string_view text);

protected:
  void init_ui();
  void deinit_ui();

  // the individual pieces of the UI, called from init_ui()
  void init_tabview();
  void init_text_tab();
  void init_lora_tab();
  void init_imu_tab();
  void init_gps_tab();
  void init_sys_tab();
  void init_help_tab();
  void init_status_bar();

  // rebuild the LoRa log label from lora_messages_; called with the mutex held
  void update_lora_log();

  // the (scrollable) content object of the active tab; called with the mutex
  // held
  lv_obj_t *active_tab_content();

  // scroll the (horizontally scrollable) tab bar so the active tab's button is
  // visible; called with the mutex held
  void scroll_active_tab_into_view();

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // LVGL objects
  lv_obj_t *tabview_{nullptr};
  lv_obj_t *text_tab_{nullptr};
  lv_obj_t *lora_tab_{nullptr};
  lv_obj_t *imu_tab_{nullptr};
  lv_obj_t *gps_tab_{nullptr};
  lv_obj_t *sys_tab_{nullptr};
  lv_obj_t *help_tab_{nullptr};
  lv_obj_t *textarea_{nullptr};
  lv_obj_t *status_label_{nullptr};
  lv_obj_t *imu_label_{nullptr};
  lv_obj_t *gps_label_{nullptr};
  lv_obj_t *sys_label_{nullptr};
  lv_obj_t *lora_status_label_{nullptr};
  lv_obj_t *lora_log_label_{nullptr};

  // recent LoRa log lines (newest at the front)
  std::deque<std::string> lora_messages_;
  static constexpr size_t MAX_LORA_MESSAGES = 8;

  static constexpr int TAB_BAR_HEIGHT = 28;
  static constexpr int STATUS_BAR_HEIGHT = 18;

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           // the tabview (nested containers + flex layout) uses
                           // more stack to render than a flat UI
                           .task_config = {.name = "gui", .stack_size_bytes = 10 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
