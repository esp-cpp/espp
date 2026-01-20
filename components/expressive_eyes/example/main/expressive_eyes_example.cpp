#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "expressive_eyes.hpp"
#include "logger.hpp"
#include "task.hpp"

// Board-specific includes based on menuconfig selection
#if CONFIG_EXPRESSIVE_EYES_BOARD_ESP_BOX
#include "esp-box.hpp"
using Board = espp::EspBox;
#elif CONFIG_EXPRESSIVE_EYES_BOARD_MATOUCH_ROTARY
#include "matouch-rotary-display.hpp"
using Board = espp::MatouchRotaryDisplay;
#elif CONFIG_EXPRESSIVE_EYES_BOARD_WROVER_KIT
#include "wrover-kit.hpp"
using Board = espp::WroverKit;
#elif CONFIG_EXPRESSIVE_EYES_BOARD_TAB5
#include "m5stack-tab5.hpp"
using Board = espp::M5StackTab5;
#else
#error "No board selected in menuconfig!"
#endif

using namespace std::chrono_literals;

static std::recursive_mutex lvgl_mutex;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Expressive Eyes Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting Expressive Eyes Example");

  //! [expressive eyes example]
  // Initialize the board
  Board &board = Board::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);

  // Initialize the LCD
  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }

  // Set up pixel buffer (50 lines)
  static constexpr size_t pixel_buffer_size = board.lcd_width() * 50;

  // Initialize the display
  if (!board.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // Get screen dimensions
  int screen_width = board.lcd_width();
  int screen_height = board.lcd_height();

  logger.info("Display size: {}x{}", screen_width, screen_height);

  // Disable scrollbars on screen
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

  // Get background color
  lv_color_t bg_color = lv_obj_get_style_bg_color(lv_screen_active(), LV_PART_MAIN);

  // Create main canvas for drawing everything
  lv_obj_t *canvas = lv_canvas_create(lv_screen_active());

  // Allocate buffer for main canvas (RGB565 = 2 bytes per pixel)
  static lv_color_t *canvas_buffer = nullptr;
  canvas_buffer = (lv_color_t *)heap_caps_malloc(screen_width * screen_height * sizeof(lv_color_t),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!canvas_buffer) {
    logger.error("Failed to allocate canvas buffer!");
    return;
  }

  lv_canvas_set_buffer(canvas, canvas_buffer, screen_width, screen_height, LV_COLOR_FORMAT_RGB565);
  lv_obj_center(canvas);

  // Store original eye dimensions for calculations
  int original_eye_height = screen_height * 0.55f;
  int eye_base_width = screen_width * 0.35f;
  int pupil_size = static_cast<int>(std::min(eye_base_width, original_eye_height) * 0.3f);

  // Drawing callback for eyes - single call for both eyes using canvas
  auto draw_eyes = [&](const espp::ExpressiveEyes::EyeState &left,
                       const espp::ExpressiveEyes::EyeState &right) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);

    // Clear canvas with background color
    lv_canvas_fill_bg(canvas, bg_color, LV_OPA_COVER);

    // Helper to draw filled ellipse (for eyes) using layer API
    auto draw_ellipse = [&](int cx, int cy, int width, int height, lv_color_t color) {
      lv_layer_t layer;
      lv_canvas_init_layer(canvas, &layer);

      lv_draw_rect_dsc_t rect_dsc;
      lv_draw_rect_dsc_init(&rect_dsc);
      rect_dsc.bg_color = color;
      rect_dsc.bg_opa = LV_OPA_COVER;
      rect_dsc.radius = LV_RADIUS_CIRCLE;
      rect_dsc.border_width = 0;

      lv_area_t area;
      area.x1 = cx - width / 2;
      area.y1 = cy - height / 2;
      area.x2 = cx + width / 2;
      area.y2 = cy + height / 2;

      lv_draw_rect(&layer, &rect_dsc, &area);
      lv_canvas_finish_layer(canvas, &layer);
    };

    // Lambda to draw single eye
    auto draw_single_eye = [&](const espp::ExpressiveEyes::EyeState &eye_state, bool is_left) {
      // Draw eye white
      draw_ellipse(eye_state.x, eye_state.y, eye_state.width, eye_state.height, lv_color_white());

      // Draw pupil (use original height so blink doesn't move pupil)
      // Only draw if eye is open enough (not blinking)
      float openness = eye_state.height / static_cast<float>(original_eye_height);
      if (eye_state.expression.pupil.enabled && openness > 0.3f) {
        int px_offset = static_cast<int>(eye_state.expression.pupil.x * eye_state.width * 0.3f);
        int py_offset = static_cast<int>(eye_state.expression.pupil.y * original_eye_height * 0.3f);
        int pupil_x = eye_state.x + px_offset;
        int pupil_y = eye_state.y + py_offset;

        lv_layer_t layer;
        lv_canvas_init_layer(canvas, &layer);

        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = lv_color_black();
        rect_dsc.bg_opa = LV_OPA_COVER;
        rect_dsc.radius = LV_RADIUS_CIRCLE;
        rect_dsc.border_width = 0;

        lv_area_t area;
        int r = pupil_size / 2;
        area.x1 = pupil_x - r;
        area.y1 = pupil_y - r;
        area.x2 = pupil_x + r;
        area.y2 = pupil_y + r;

        lv_draw_rect(&layer, &rect_dsc, &area);
        lv_canvas_finish_layer(canvas, &layer);
      }

      // Draw eyebrow as rotated rectangle (using triangles)
      if (eye_state.expression.eyebrow.enabled) {
        int brow_width =
            static_cast<int>(eye_base_width * eye_state.expression.eyebrow.width * 1.5f);
        int brow_height =
            static_cast<int>(eye_base_width * eye_state.expression.eyebrow.thickness * 4.0f);
        int brow_y = eye_state.y - static_cast<int>(original_eye_height * 0.4f);

        // For left eye, positive angle tilts left side down (clockwise rotation)
        // For right eye, positive angle tilts right side down (counter-clockwise rotation)
        float angle_rad = eye_state.expression.eyebrow.angle * M_PI / 180.0f;
        if (!is_left)
          angle_rad = -angle_rad; // Mirror for right eye

        // Calculate the 4 corners of a rotated rectangle centered at (eye_state.x, brow_y)
        float half_w = brow_width / 2.0f;
        float half_h = brow_height / 2.0f;
        float cos_a = std::cos(angle_rad);
        float sin_a = std::sin(angle_rad);

        // Four corners relative to center, then rotated
        int x1 = eye_state.x + static_cast<int>(-half_w * cos_a + half_h * sin_a);
        int y1 = brow_y + static_cast<int>(-half_w * sin_a - half_h * cos_a);

        int x2 = eye_state.x + static_cast<int>(half_w * cos_a + half_h * sin_a);
        int y2 = brow_y + static_cast<int>(half_w * sin_a - half_h * cos_a);

        int x3 = eye_state.x + static_cast<int>(half_w * cos_a - half_h * sin_a);
        int y3 = brow_y + static_cast<int>(half_w * sin_a + half_h * cos_a);

        int x4 = eye_state.x + static_cast<int>(-half_w * cos_a - half_h * sin_a);
        int y4 = brow_y + static_cast<int>(-half_w * sin_a + half_h * cos_a);

        lv_layer_t layer;
        lv_canvas_init_layer(canvas, &layer);

        lv_draw_triangle_dsc_t tri_dsc;
        lv_draw_triangle_dsc_init(&tri_dsc);
        tri_dsc.color = bg_color;
        tri_dsc.opa = LV_OPA_COVER;

        // First triangle: top-left, top-right, bottom-right
        tri_dsc.p[0].x = x1;
        tri_dsc.p[0].y = y1;
        tri_dsc.p[1].x = x2;
        tri_dsc.p[1].y = y2;
        tri_dsc.p[2].x = x3;
        tri_dsc.p[2].y = y3;
        lv_draw_triangle(&layer, &tri_dsc);

        // Second triangle: top-left, bottom-right, bottom-left
        tri_dsc.p[0].x = x1;
        tri_dsc.p[0].y = y1;
        tri_dsc.p[1].x = x3;
        tri_dsc.p[1].y = y3;
        tri_dsc.p[2].x = x4;
        tri_dsc.p[2].y = y4;
        lv_draw_triangle(&layer, &tri_dsc);

        lv_canvas_finish_layer(canvas, &layer);
      }

      // Draw cheek
      if (eye_state.expression.cheek.enabled) {
        int cheek_width = static_cast<int>(eye_base_width * eye_state.expression.cheek.size * 2.5f);
        int cheek_height =
            static_cast<int>(eye_base_width * eye_state.expression.cheek.size * 1.5f);
        int cheek_y = eye_state.y + static_cast<int>(original_eye_height * 0.4f) +
                      static_cast<int>(eye_state.expression.cheek_offset_y * screen_height);

        lv_layer_t layer;
        lv_canvas_init_layer(canvas, &layer);

        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = bg_color;
        rect_dsc.bg_opa = LV_OPA_COVER;
        rect_dsc.radius = LV_RADIUS_CIRCLE;
        rect_dsc.border_width = 0;

        lv_area_t area;
        area.x1 = eye_state.x - cheek_width / 2;
        area.y1 = cheek_y - cheek_height / 2;
        area.x2 = eye_state.x + cheek_width / 2;
        area.y2 = cheek_y + cheek_height / 2;

        lv_draw_rect(&layer, &rect_dsc, &area);
        lv_canvas_finish_layer(canvas, &layer);
      }
    };

    // Draw both eyes
    draw_single_eye(left, true);
    draw_single_eye(right, false);
  };

  // Configure expressive eyes with adaptive sizing - make eyes larger
  int large_eye_width = screen_width * 0.35f;   // 35% of screen width each
  int large_eye_height = screen_height * 0.55f; // 55% of screen height
  int large_spacing = screen_width * 0.55f;     // Space between eye centers

  espp::ExpressiveEyes::Config config{.screen_width = screen_width,
                                      .screen_height = screen_height,
                                      .eye_spacing = large_spacing,
                                      .eye_width = large_eye_width,
                                      .eye_height = large_eye_height,
                                      .eye_color = 0xFFFF,
                                      .blink_duration = 0.12f,
                                      .blink_interval = 4.0f,
                                      .enable_auto_blink = true,
                                      .enable_pupil_physics = true,
                                      .on_draw = draw_eyes,
                                      .log_level = espp::Logger::Verbosity::WARN};

  espp::ExpressiveEyes eyes(config);

  // start a simple thread to do the lv_task_handler every 16ms
  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        {
                          std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
                          lv_task_handler();
                        }
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {
                          .name = "lv_task",
                          .stack_size_bytes = 16 * 1024,
                      }});
  lv_task.start();

  logger.info("Expressive eyes initialized");

  // Test different expressions using array iteration
  logger.info("Testing different expressions...");

  const espp::ExpressiveEyes::Expression expressions[] = {
      espp::ExpressiveEyes::Expression::NEUTRAL, espp::ExpressiveEyes::Expression::HAPPY,
      espp::ExpressiveEyes::Expression::SAD, espp::ExpressiveEyes::Expression::ANGRY,
      espp::ExpressiveEyes::Expression::SURPRISED};

  auto last_time = std::chrono::steady_clock::now();
  for (const auto &expr : expressions) {
    logger.info("Expression: {}", espp::ExpressiveEyes::expression_name(expr));
    eyes.set_expression(expr);
    for (int i = 0; i < 180; i++) { // 3 seconds at 60fps
      auto now = std::chrono::steady_clock::now();
      float dt = std::chrono::duration<float>(now - last_time).count();
      last_time = now;
      eyes.update(dt);
      std::this_thread::sleep_for(16ms);
    }
  }

  // Test look_at functionality using array iteration
  logger.info("Testing look_at functionality");
  eyes.set_expression(espp::ExpressiveEyes::Expression::NEUTRAL);

  struct LookDirection {
    const char *name;
    float x;
    float y;
  };

  const LookDirection look_directions[] = {{"left", -1.0f, 0.0f},
                                           {"right", 1.0f, 0.0f},
                                           {"up", 0.0f, -1.0f},
                                           {"down", 0.0f, 1.0f},
                                           {"center", 0.0f, 0.0f}};

  for (const auto &dir : look_directions) {
    logger.info("Looking {}", dir.name);
    eyes.look_at(dir.x, dir.y);
    for (int i = 0; i < 90; i++) { // 1.5 seconds at 60fps (faster)
      auto now = std::chrono::steady_clock::now();
      float dt = std::chrono::duration<float>(now - last_time).count();
      last_time = now;
      eyes.update(dt);
      std::this_thread::sleep_for(16ms);
    }
  }

  // Back to normal with continuous updates
  logger.info("Expression: Normal (continuous loop)");

  // Animation loop
  last_time = std::chrono::steady_clock::now();
  while (true) {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    // Update and render eyes (calls draw callback)
    eyes.update(dt);

    std::this_thread::sleep_for(16ms); // ~60 FPS
  }
  //! [expressive eyes example]

  logger.info("Expressive Eyes example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
