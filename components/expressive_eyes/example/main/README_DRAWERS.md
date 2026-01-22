# Eye Drawer Implementations

This directory contains different implementations for drawing expressive eyes. The drawing logic has been separated into modular components to demonstrate different visual styles and allow easy customization.

## Architecture

The drawer system uses a base interface (`EyeDrawer`) with concrete implementations:

- **`eye_drawer.hpp`**: Base interface that all drawers must implement
- **`full_featured_drawer.hpp`**: Traditional realistic eyes with white background and black pupils
- **`monochrome_blue_drawer.hpp`**: Electric blue eyes on black background (no pupils)

## Available Drawers

### Full Featured Drawer

Draws realistic-looking eyes with:
- White eye background
- Black pupils with position tracking
- Eyebrows with rotation support
- Cheeks for emotional expressions
- Uses background color from LVGL theme

**Best for**: Traditional robot/character displays, realistic eye expressions

### Monochrome Blue Drawer

Draws minimalist electric blue eyes with:
- Electric blue (#00BFFF) eye shapes
- Black background
- No pupils (solid colored eyes)
- Black eyebrows and cheeks (cut out from eyes as negative space)
- Clean, modern aesthetic

**Best for**: Sci-fi/futuristic interfaces, minimal power displays, WS-S3-Touch boards

## Configuration

The drawing method is selected via menuconfig:
```
Expressive Eyes Example Configuration > Select Drawing Method
```

Options:
- **Full Featured** (default for most boards)
- **Monochrome Blue** (default for WS-S3-Touch)

## Adding New Drawers

To create a new drawer implementation:

1. Create a new header file (e.g., `my_custom_drawer.hpp`)
2. Inherit from `eye_drawer::EyeDrawer`
3. Implement required methods:
   - `get_draw_callback()`: Return the drawing function
   - `cleanup()`: Clean up resources if needed
4. Add config option to `Kconfig.projbuild`
5. Add conditional compilation to `expressive_eyes_example.cpp`

### Example Template

```cpp
#pragma once
#include "eye_drawer.hpp"

namespace eye_drawer {

class MyCustomDrawer : public EyeDrawer {
public:
  struct Config {
    int screen_width;
    int screen_height;
    lv_obj_t *canvas;
    lv_color_t *canvas_buffer;
    std::recursive_mutex &lvgl_mutex;
  };

  explicit MyCustomDrawer(const Config &config) { /* ... */ }
  
  std::function<void(const espp::ExpressiveEyes::EyeState &,
                     const espp::ExpressiveEyes::EyeState &)>
  get_draw_callback() override {
    return [this](const espp::ExpressiveEyes::EyeState &left,
                  const espp::ExpressiveEyes::EyeState &right) {
      // Your drawing code here
    };
  }
  
  void cleanup() override { /* ... */ }
};

} // namespace eye_drawer
```

## Drawing Tips

- Use `lv_canvas_init_layer()` and `lv_canvas_finish_layer()` for each drawing operation
- Access eye state through `EyeState` structure (position, size, expression)
- Lock `lvgl_mutex` before any LVGL operations
- Use `eye_state.height / original_eye_height` to detect blink state
- Mirror eyebrow angles for left vs right eyes
- Use `screen_width` and `screen_height` for relative positioning
