#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>

#include "smartpanlee-sc01-plus.hpp"

#include "logger.hpp"
#include "task.hpp"

/// The Gui class encapsulates all of the LVGL UI for this example. It follows
/// the recommended pattern for building UIs with espp + LVGL in C++:
///
/// * All LVGL objects are created in init_ui(), which is broken into small
///   member functions - one per logical piece of the UI.
/// * The class owns the task which calls lv_task_handler(), and a recursive
///   mutex which guards every LVGL call. Public methods lock that mutex, so
///   other tasks (touch callbacks, sensor tasks, etc.) can safely call them.
/// * LVGL event callbacks are registered with `this` as the user-data and
///   dispatched through a single static trampoline (event_callback) into
///   member functions, keeping all UI logic inside the class.
///
/// For this example the Gui shows:
/// The UI is organized as a tabview so each subsystem gets its own
/// uncrowded page:
/// * "Draw" tab: instructions, plus the rotate and clear buttons. Touching
///   the screen while this tab is active draws circles (on a transparent
///   overlay) and plays a click.
/// * "Audio" tab: a play-sound button (wired to the example via a
///   callback), a mute toggle, and volume down / up buttons (acting
///   directly on the BSP), with a label showing the current volume.
class Gui {
public:
  /// Callback invoked when the play-sound button is pressed
  using audio_button_callback_t = std::function<void()>;
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

  /// Set the text of the info label. Thread-safe.
  /// @param text The text to display
  void set_label_text(std::string_view text);

  /// Draw a circle at the given screen coordinates, replacing the oldest
  /// circle if the maximum number are already visible. Thread-safe.
  /// @param x The x coordinate (screen space)
  /// @param y The y coordinate (screen space)
  /// @param radius The radius of the circle
  void draw_circle(int x, int y, int radius);

  /// Clear all circles from the screen. Thread-safe.
  void clear_circles();

  /// Rotate the display to the next of 0/90/180/270 degrees, resizing /
  /// re-aligning the UI to match. Thread-safe.
  void next_rotation();

  /// Whether the Draw tab is currently active (used by the example to only
  /// draw circles / play clicks for touches on that tab). Thread-safe.
  /// @return True if the Draw tab is the active tab
  bool draw_page_active();

  /// Set the callback invoked when the play-sound button is pressed
  /// @param callback The callback to invoke
  void set_play_callback(audio_button_callback_t callback) { play_callback_ = std::move(callback); }

protected:
  static constexpr size_t MAX_CIRCLES = 100;
  static constexpr int TAB_BAR_HEIGHT = 44;

  struct Circle {
    int x{0};
    int y{0};
    int radius{0};
    bool visible{false};
  };

  void init_ui();
  void deinit_ui();

  // the individual pieces of the UI, called from init_ui()
  void init_tabview();
  void init_label();
  void init_buttons();
  void init_audio_controls();
  void init_circle_layer();

  // update the audio volume label from the BSP's current volume / mute
  // state; called with the mutex held
  void update_audio_label();

  // re-size / re-align the UI to fill the (possibly rotated) display
  void update_layout();

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // single trampoline for all LVGL events; dispatches to the member
  // functions below based on the event target
  static void event_callback(lv_event_t *e);
  void on_pressed(lv_event_t *e);
  void on_tab_changed(lv_event_t *e);

  // custom drawing of the circle layer
  static void draw_circle_layer(lv_event_t *e);
  void draw_circles(lv_event_t *e) const;
  void invalidate_circle_area(const Circle &circle);

  // unlocked implementations, called with the mutex held
  void clear_circles_impl();

  // LVGL objects
  lv_obj_t *tabview_{nullptr};
  lv_obj_t *draw_tab_{nullptr};
  lv_obj_t *audio_tab_{nullptr};
  lv_obj_t *label_{nullptr};
  lv_obj_t *rotate_button_{nullptr};
  lv_obj_t *clear_button_{nullptr};
  lv_obj_t *play_button_{nullptr};
  lv_obj_t *mute_button_{nullptr};
  lv_obj_t *volume_down_button_{nullptr};
  lv_obj_t *volume_up_button_{nullptr};
  lv_obj_t *audio_label_{nullptr};
  // last values shown on audio_label_, so update_audio_label() (called every
  // GUI tick) only reformats the text when a value actually changes
  int last_volume_{-1};
  bool last_muted_{false};
  lv_obj_t *circle_layer_{nullptr};

  audio_button_callback_t play_callback_{nullptr};

  std::array<Circle, MAX_CIRCLES> circles_;
  size_t next_circle_index_{0};
  size_t visible_circle_count_{0};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           // NOTE: rendering the tabview (nested containers + flex layout) uses
                           // noticeably more stack than a flat UI; 6 KB overflows
                           .task_config = {.name = "gui", .stack_size_bytes = 12 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
