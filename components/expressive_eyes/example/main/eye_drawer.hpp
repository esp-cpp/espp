#pragma once

#include "expressive_eyes.hpp"
#include <functional>

#include <lvgl.h>

namespace eye_drawer {

/**
 * @brief Base interface for eye drawing implementations
 *
 * Defines the interface that all eye drawer implementations must follow.
 * Drawer implementations receive eye state data from ExpressiveEyes and
 * are responsible for rendering the eyes using their chosen graphics API.
 *
 * @note Implementations should use thread-safe operations when drawing
 *       (e.g., lock LVGL mutex before LVGL calls).
 */
struct EyeDrawer {
  virtual ~EyeDrawer() = default;

  /**
   * @brief Callback function type for drawing eyes
   * @param left_eye Left eye state data
   * @param right_eye Right eye state data
   */
  typedef std::function<void(const espp::ExpressiveEyes::EyeState &,
                             const espp::ExpressiveEyes::EyeState &)>
      DrawCallback;

  /**
   * @brief Get the draw callback function
   * @return The callback function to be used with ExpressiveEyes
   */
  virtual DrawCallback get_draw_callback() = 0;

  /**
   * @brief Clean up resources
   *
   * Called when the drawer is being destroyed. Implementations should
   * free any allocated resources.
   */
  virtual void cleanup() = 0;
};

} // namespace eye_drawer
