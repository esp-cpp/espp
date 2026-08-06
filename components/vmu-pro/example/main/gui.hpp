#pragma once

#include <array>
#include <mutex>
#include <string>
#include <string_view>

#include "vmu-pro.hpp"

#include "logger.hpp"
#include "task.hpp"

/// The Gui class encapsulates all of the LVGL UI for this example. It follows
/// the recommended pattern for building UIs with espp + LVGL in C++:
///
/// * All LVGL objects are created in init_ui(), which is broken into small
///   member functions - one per logical piece of the UI.
/// * The class owns the task which calls lv_task_handler(), and a recursive
///   mutex which guards every LVGL call. Public methods lock that mutex, so
///   other tasks (button callbacks, etc.) can safely call them.
///
/// Since the VMU Pro has no touch screen, the UI is driven entirely by the
/// buttons through the public methods below. For this example the Gui shows:
/// * A label with instructions and the most recent button event
/// * A cursor which is moved around the screen with the D-pad
/// * A custom-drawn layer of circles drawn at the cursor with the A button
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

  /// Move the cursor by the given amount, clamping it to the screen bounds.
  /// Thread-safe.
  /// @param dx The change in x position, in pixels
  /// @param dy The change in y position, in pixels
  void move_cursor(int dx, int dy);

  /// Draw a circle at the current cursor position, replacing the oldest
  /// circle if the maximum number are already visible. Thread-safe.
  void draw_at_cursor();

  /// Clear all circles from the screen. Thread-safe.
  void clear_circles();

  /// Rotate the display to the next of 0/90/180/270 degrees, resizing /
  /// re-aligning the UI to match. Thread-safe.
  void next_rotation();

  /// Cycle the backlight brightness through 25/50/75/100%. Thread-safe.
  void cycle_brightness();

protected:
  static constexpr size_t MAX_CIRCLES = 100;
  static constexpr int CURSOR_RADIUS = 8;
  static constexpr int CIRCLE_RADIUS = 10;
  static constexpr std::array<float, 4> BRIGHTNESS_LEVELS = {25.0f, 50.0f, 75.0f, 100.0f};

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
  void init_cursor();
  void init_circle_layer();

  // re-position the cursor object to match cursor_x_/cursor_y_
  void update_cursor();

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // custom drawing of the circle layer
  static void draw_circle_layer(lv_event_t *e);
  void draw_circles(lv_event_t *e) const;
  void invalidate_circle_area(const Circle &circle);

  // unlocked implementations, called with the mutex held
  void draw_circle_impl(int x, int y, int radius);
  void clear_circles_impl();

  // LVGL objects
  lv_obj_t *background_{nullptr};
  lv_obj_t *label_{nullptr};
  lv_obj_t *cursor_{nullptr};
  lv_obj_t *circle_layer_{nullptr};

  int cursor_x_{0};
  int cursor_y_{0};

  size_t brightness_index_{2}; // start at 75%

  std::array<Circle, MAX_CIRCLES> circles_;
  size_t next_circle_index_{0};
  size_t visible_circle_count_{0};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 6 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
