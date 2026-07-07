#pragma once

#include <array>
#include <mutex>
#include <string>
#include <string_view>

#include "wrover-kit.hpp"

#include "logger.hpp"
#include "task.hpp"

/// The Gui class encapsulates all of the LVGL UI for this example. It follows
/// the recommended pattern for building UIs with espp + LVGL in C++:
///
/// * All LVGL objects are created in init_ui(), which is broken into small
///   member functions - one per logical piece of the UI.
/// * The class owns the task which calls lv_task_handler(), and a recursive
///   mutex which guards every LVGL call. Public methods lock that mutex, so
///   other tasks (button callbacks, the drawing loop, etc.) can safely call
///   them.
///
/// For this example the Gui shows:
/// * A label in the center of the screen
/// * A custom-drawn layer of circles which the main loop draws automatically
///
/// The Wrover-Kit has no touchscreen, so there are no on-screen buttons;
/// instead the boot button cycles the display rotation via next_rotation().
class Gui {
public:
  /// The maximum number of circles which can be visible at once; the main
  /// loop clears the circles once this many have been drawn.
  static constexpr size_t MAX_CIRCLES = 10;

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

  /// Draw a circle at the given screen coordinates, replacing the oldest
  /// circle if the maximum number are already visible. Thread-safe.
  /// @param x The x coordinate (screen space)
  /// @param y The y coordinate (screen space)
  /// @param radius The radius of the circle
  void draw_circle(int x, int y, int radius);

  /// Clear all circles from the screen. Thread-safe.
  void clear_circles();

  /// Get the number of circles currently visible. Thread-safe.
  /// @return The number of visible circles
  size_t circle_count();

  /// Rotate the display to the next of 0/90/180/270 degrees, resizing /
  /// re-aligning the UI to match. Thread-safe.
  void next_rotation();

protected:
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
  void init_circle_layer();

  // the LVGL update task: calls lv_task_handler() under the mutex
  bool update(std::mutex &m, std::condition_variable &cv);

  // custom drawing of the circle layer
  static void draw_circle_layer(lv_event_t *e);
  void draw_circles(lv_event_t *e) const;
  void invalidate_circle_area(const Circle &circle);

  // unlocked implementations, called with the mutex held
  void clear_circles_impl();

  // LVGL objects
  lv_obj_t *background_{nullptr};
  lv_obj_t *label_{nullptr};
  lv_obj_t *circle_layer_{nullptr};

  std::array<Circle, MAX_CIRCLES> circles_;
  size_t next_circle_index_{0};
  size_t visible_circle_count_{0};

  espp::Task update_task_{{.callback = [this](auto &m, auto &cv) { return update(m, cv); },
                           .task_config = {.name = "gui", .stack_size_bytes = 6 * 1024}}};
  espp::Logger logger_;
  std::recursive_mutex mutex_;
};
