#pragma once

#include "expressive_eyes.hpp"
#include <functional>

#include <lvgl.h>

namespace eye_drawer {

/// \brief Base interface for eye drawing implementations
struct EyeDrawer {
  virtual ~EyeDrawer() = default;

  typedef std::function<void(const espp::ExpressiveEyes::EyeState &,
                             const espp::ExpressiveEyes::EyeState &)>
      DrawCallback;

  /// \brief Get the draw callback function
  /// \return The callback function to be used with ExpressiveEyes
  virtual DrawCallback get_draw_callback() = 0;

  /// \brief Clean up resources
  virtual void cleanup() = 0;
};

} // namespace eye_drawer
