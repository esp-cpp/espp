# Expressive Eyes Component

Animated expressive eyes for displays using simple blob shapes.

## Features

- Multiple expressions (happy, sad, angry, surprised, sleepy, wink)
- Smooth eye movement and blinking
- Physics-based pupil movement
- Customizable appearance
- Frame-based animation system
- Automatic random blinking

## Usage

```cpp
//! [expressive eyes example]
#include "expressive_eyes.hpp"
#include "display.hpp"  // Your display driver

// Drawing callback
auto draw_eye = [](int x, int y, int width, int height, float rotation,
                   uint16_t color, bool has_pupil, float pupil_x, float pupil_y) {
    // Draw eye ellipse
    display.fill_ellipse(x, y, width/2, height/2, color);
    
    if (has_pupil && height > 10) {
        // Calculate pupil position
        int px = x + static_cast<int>(pupil_x * width * 0.3f);
        int py = y + static_cast<int>(pupil_y * height * 0.3f);
        display.fill_circle(px, py, 10, 0x0000);  // Black pupil
    }
};

// Configure eyes
espp::ExpressiveEyes::Config config{
    .screen_width = 320,
    .screen_height = 240,
    .eye_spacing = 100,
    .eye_width = 60,
    .eye_height = 80,
    .pupil_size = 20,
    .enable_auto_blink = true,
    .enable_pupil_physics = true,
    .on_draw = draw_eye
};

espp::ExpressiveEyes eyes(config);

// Animation loop
auto last_time = std::chrono::steady_clock::now();
while (true) {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;
    
    eyes.update(dt);
    
    // Change expression based on input
    eyes.set_expression(espp::ExpressiveEyes::Expression::HAPPY);
    
    // Look at position
    eyes.look_at(0.5f, -0.3f);  // Look up-right
    
    std::this_thread::sleep_for(16ms);  // ~60 FPS
}
//! [expressive eyes example]
```

## Expressions

- `NEUTRAL` - Normal open eyes
- `HAPPY` - Squinted happy eyes
- `SAD` - Droopy sad eyes
- `ANGRY` - Angled angry eyebrows
- `SURPRISED` - Wide open eyes
- `SLEEPY` - Half-closed eyes
- `WINK_LEFT` - Left eye closed
- `WINK_RIGHT` - Right eye closed

## Customization

- Adjust `eye_width` and `eye_height` for different eye shapes
- Set `pupil_size = 0` to disable pupils
- Modify `blink_interval` to change blink frequency
- Disable `enable_pupil_physics` for instant pupil movement
