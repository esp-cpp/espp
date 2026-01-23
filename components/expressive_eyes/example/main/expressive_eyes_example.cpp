#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <vector>

#include "expressive_eyes.hpp"
#include "logger.hpp"
#include "task.hpp"

// Include drawer implementations
#include "full_featured_drawer.hpp"
#include "monochrome_blue_drawer.hpp"

// Board-specific includes based on menuconfig selection
#if CONFIG_EXPRESSIVE_EYES_BOARD_ESP_BOX
#include "esp-box.hpp"
using Board = espp::EspBox;
#elif CONFIG_EXPRESSIVE_EYES_BOARD_MATOUCH_ROTARY
#include "matouch-rotary-display.hpp"
using Board = espp::MatouchRotaryDisplay;
#elif CONFIG_EXPRESSIVE_EYES_BOARD_WS_S3_TOUCH
#include "ws-s3-touch.hpp"
using Board = espp::WsS3Touch;
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

  // Create the drawer based on menuconfig selection
  std::unique_ptr<eye_drawer::EyeDrawer> drawer;

#if CONFIG_EXPRESSIVE_EYES_FULL_FEATURED
  logger.info("Using Full Featured drawer");
  drawer = std::make_unique<eye_drawer::FullFeaturedDrawer>(
      eye_drawer::FullFeaturedDrawer::Config{.screen_width = screen_width,
                                             .screen_height = screen_height,
                                             .canvas = canvas,
                                             .canvas_buffer = canvas_buffer,
                                             .lvgl_mutex = lvgl_mutex});
#elif CONFIG_EXPRESSIVE_EYES_MONOCHROME_BLUE
  logger.info("Using Monochrome Blue drawer");
  drawer = std::make_unique<eye_drawer::MonochromeBlueDrawer>(
      eye_drawer::MonochromeBlueDrawer::Config{.screen_width = screen_width,
                                               .screen_height = screen_height,
                                               .canvas = canvas,
                                               .canvas_buffer = canvas_buffer,
                                               .lvgl_mutex = lvgl_mutex});
#else
#error "No drawing method selected in menuconfig!"
#endif

  // Get the draw callback from the drawer
  auto draw_eyes = drawer->get_draw_callback();

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

  // Random demo mode - continuously looks around and changes expressions
  logger.info("Starting random demo mode - will run continuously");

  // Seed random number generator
  srand(time(nullptr));

  // Reset to neutral
  eyes.set_expression(espp::ExpressiveEyes::Expression::NEUTRAL);
  eyes.look_at(0.0f, 0.0f);

  // Random mode state
  float time_until_next_look = 2.0f + (rand() % 4000) / 1000.0f;        // 2-6 seconds
  float time_until_next_expression = 5.0f + (rand() % 10000) / 1000.0f; // 5-15 seconds
  float look_timer = 0.0f;
  float expression_timer = 0.0f;

  // Animation loop
  last_time = std::chrono::steady_clock::now();
  while (true) {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    // Update timers
    look_timer += dt;
    expression_timer += dt;

    // Randomly look around
    if (look_timer >= time_until_next_look) {
      float look_x = ((rand() % 2000) - 1000) / 1000.0f; // -1.0 to 1.0
      float look_y = ((rand() % 2000) - 1000) / 1000.0f; // -1.0 to 1.0
      eyes.look_at(look_x, look_y);
      look_timer = 0.0f;
      time_until_next_look = 2.0f + (rand() % 4000) / 1000.0f; // 2-6 seconds
    }

    // Randomly change expression (weighted toward neutral)
    if (expression_timer >= time_until_next_expression) {
      int expr_choice = rand() % 10;
      if (expr_choice < 5) {
        // 50% chance: stay neutral
        eyes.set_expression(espp::ExpressiveEyes::Expression::NEUTRAL);
      } else if (expr_choice < 7) {
        // 20% chance: happy
        eyes.set_expression(espp::ExpressiveEyes::Expression::HAPPY);
      } else if (expr_choice < 8) {
        // 10% chance: surprised
        eyes.set_expression(espp::ExpressiveEyes::Expression::SURPRISED);
      } else if (expr_choice < 9) {
        // 10% chance: sad
        eyes.set_expression(espp::ExpressiveEyes::Expression::SAD);
      } else {
        // 10% chance: angry
        eyes.set_expression(espp::ExpressiveEyes::Expression::ANGRY);
      }
      expression_timer = 0.0f;
      time_until_next_expression = 5.0f + (rand() % 10000) / 1000.0f; // 5-15 seconds
    }

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
