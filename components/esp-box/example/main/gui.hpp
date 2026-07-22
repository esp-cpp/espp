#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>

#include "esp-box.hpp"

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
/// The UI is organized as a tabview so each subsystem gets its own
/// uncrowded page:
/// * "Draw" tab: instructions, plus the rotate and clear buttons. Touching
///   the screen while this tab is active draws circles (on a transparent
///   overlay) and plays a click.
/// * "IMU" tab: live IMU text and two lines showing the direction of
///   gravity ("down") as computed by a Kalman filter (blue) and a Madgwick
///   filter (red).
/// * "Audio" tab: record / play buttons (wired to the example via
///   callbacks) and speaker / microphone volume buttons (acting directly on
///   the BSP), with a label showing the current volumes.
class Gui {
public:
  /// Callback invoked when the record / play buttons are pressed
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

  /// Set the instruction text shown on the Draw tab. Thread-safe.
  /// @param text The text to display
  void set_label_text(std::string_view text);

  /// Set the live IMU text shown on the IMU tab. Thread-safe.
  /// @param text The text to display
  void set_imu_text(std::string_view text);

  /// Whether the Draw tab is currently active (used by the example to only
  /// draw circles / play clicks for touches on that tab). Thread-safe.
  /// @return True if the Draw tab is the active tab
  bool draw_page_active();

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

  /// Update the line showing "down" according to the Kalman filter. The
  /// vector is provided in the display's natural (unrotated) frame; the Gui
  /// accounts for the current display rotation. Thread-safe.
  /// @param vx The x component of the gravity vector
  /// @param vy The y component of the gravity vector
  void set_kalman_down(float vx, float vy);

  /// Update the line showing "down" according to the Madgwick filter. The
  /// vector is provided in the display's natural (unrotated) frame; the Gui
  /// accounts for the current display rotation. Thread-safe.
  /// @param vx The x component of the gravity vector
  /// @param vy The y component of the gravity vector
  void set_madgwick_down(float vx, float vy);

  /// Set the callback invoked when the record button is pressed
  /// @param callback The callback to invoke
  void set_record_callback(audio_button_callback_t callback) {
    record_callback_ = std::move(callback);
  }

  /// Set the callback invoked when the play button is pressed
  /// @param callback The callback to invoke
  void set_play_callback(audio_button_callback_t callback) { play_callback_ = std::move(callback); }

  /// Show whether a recording is in progress (turns the record button red
  /// and changes its symbol to stop). Thread-safe.
  /// @param active True while recording
  void set_record_active(bool active);

  /// Show whether a playback is in progress (turns the play button green and
  /// changes its symbol to stop). Thread-safe.
  /// @param active True while playing
  void set_play_active(bool active);

  /// Set the status line on the Audio tab (e.g. "Recording...", "Mic
  /// unavailable"). Thread-safe.
  /// @param text The text to display
  void set_audio_status(std::string_view text);

protected:
  static constexpr size_t MAX_CIRCLES = 100;
  static constexpr int TAB_BAR_HEIGHT = 40;

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
  void init_draw_tab();
  void init_imu_tab();
  void init_audio_tab();
  void init_circle_layer();

  // update the audio volume label from the BSP's current volumes; called
  // with the mutex held
  void update_audio_label();

  // update the given "down" line from a vector in the unrotated display
  // frame, remapping for the current display rotation; called with the
  // mutex held
  void set_down_line(lv_obj_t *line, lv_point_precise_t *points, float vx, float vy);

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
  lv_obj_t *imu_tab_{nullptr};
  lv_obj_t *audio_tab_{nullptr};
  lv_obj_t *label_{nullptr};
  lv_obj_t *imu_label_{nullptr};
  lv_obj_t *kalman_line_{nullptr};
  lv_obj_t *madgwick_line_{nullptr};
  lv_obj_t *rotate_button_{nullptr};
  lv_obj_t *clear_button_{nullptr};
  lv_obj_t *record_button_{nullptr};
  lv_obj_t *record_button_label_{nullptr};
  lv_obj_t *play_button_{nullptr};
  lv_obj_t *play_button_label_{nullptr};
  lv_obj_t *volume_down_button_{nullptr};
  lv_obj_t *volume_up_button_{nullptr};
  lv_obj_t *mic_down_button_{nullptr};
  lv_obj_t *mic_up_button_{nullptr};
  lv_obj_t *audio_label_{nullptr};
  lv_obj_t *audio_status_label_{nullptr};
  lv_obj_t *circle_layer_{nullptr};

  audio_button_callback_t record_callback_{nullptr};
  audio_button_callback_t play_callback_{nullptr};

  lv_style_t kalman_line_style_;
  lv_style_t madgwick_line_style_;
  lv_point_precise_t kalman_line_points_[2];
  lv_point_precise_t madgwick_line_points_[2];

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
