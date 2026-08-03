#pragma once

#include <array>
#include <mutex>
#include <string>
#include <string_view>

#include "ws-s3-touch.hpp"

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
/// * A label with instructions and live IMU data
/// * A label at the top of the screen showing the current RTC time
/// * Two lines showing the direction of gravity ("down") as computed by a
///   Kalman filter (blue) and a Madgwick filter (red)
/// * A button (top-left) which rotates the display through 0/90/180/270
/// * A button (top-right) which clears the circles drawn on the screen
/// * A custom-drawn layer of circles which trail the most recent touches
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

  /// Set the text of the main label. Thread-safe.
  /// @param text The text to display
  void set_label_text(std::string_view text);

  /// Set the text of the clock label at the top of the screen. Thread-safe.
  /// @param text The text to display
  void set_clock_text(std::string_view text);

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

protected:
  using Bsp = espp::WsS3Touch;

  static constexpr size_t MAX_CIRCLES = 100;

  struct Circle {
    int x{0};
    int y{0};
    int radius{0};
    bool visible{false};
  };

  void init_ui();
  void deinit_ui();

  // the individual pieces of the UI, called from init_ui()
  void init_background();
  void init_label();
  void init_clock_label();
  void init_gravity_lines();
  void init_buttons();
  void init_circle_layer();

  // resize / re-align the UI to match the current display rotation
  void update_layout();

  // update the given "down" line from a vector in the unrotated display
  // frame, remapping for the current display rotation
  void set_down_line(lv_obj_t *line, lv_point_precise_t *points, float vx, float vy);

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // single trampoline for all LVGL events; dispatches to the member
  // functions below based on the event target
  static void event_callback(lv_event_t *e);
  void on_pressed(lv_event_t *e);

  // custom drawing of the circle layer
  static void draw_circle_layer(lv_event_t *e);
  void draw_circles(lv_event_t *e) const;
  void invalidate_circle_area(const Circle &circle);

  // unlocked implementations, called with the mutex held
  void clear_circles_impl();

  // LVGL objects
  lv_obj_t *background_{nullptr};
  lv_obj_t *label_{nullptr};
  lv_obj_t *clock_label_{nullptr};
  lv_obj_t *kalman_line_{nullptr};
  lv_obj_t *madgwick_line_{nullptr};
  lv_obj_t *rotate_button_{nullptr};
  lv_obj_t *clear_button_{nullptr};
  lv_obj_t *circle_layer_{nullptr};

  lv_style_t kalman_line_style_;
  lv_style_t madgwick_line_style_;
  lv_point_precise_t kalman_line_points_[2];
  lv_point_precise_t madgwick_line_points_[2];

  std::array<Circle, MAX_CIRCLES> circles_;
  size_t next_circle_index_{0};
  size_t visible_circle_count_{0};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 6 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
