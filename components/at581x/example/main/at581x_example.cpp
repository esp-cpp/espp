#include <atomic>
#include <chrono>
#include <memory>
#include <sdkconfig.h>
#include <string>
#include <thread>

#include "at581x.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "logger.hpp"
#include "task.hpp"

#if defined(CONFIG_EXAMPLE_HARDWARE_BOX3_SENSOR)
#include <mutex>

#include "esp-box.hpp"
#endif

using namespace std::chrono_literals;

// The ESP32-S3-BOX-3 BSP uses I2C_NUM_0 (GPIO8/18) for its internal bus, so put the dock radar
// (GPIO41/40) on a different port when targeting the box-3.
#if defined(CONFIG_EXAMPLE_HARDWARE_BOX3_SENSOR)
static constexpr i2c_port_t RADAR_I2C_PORT = I2C_NUM_1;
#else
static constexpr i2c_port_t RADAR_I2C_PORT = I2C_NUM_0;
#endif

// Live radar state shared between the interrupt callback, the display, and the logging loop.
static std::atomic<bool> g_presence{false};
static std::atomic<int> g_presence_count{0};
static std::atomic<int> g_sensing_distance{700};

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "at581x example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting AT581X radar example");

  //! [at581x example]
  // Make the I2C bus the radar is on (SDA=41, SCL=40 on the ESP32-S3-BOX-3 sensor dock).
  espp::I2c i2c({
      .port = RADAR_I2C_PORT,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });

  std::error_code ec;
  auto radar_dev = i2c.add_device<uint8_t>({.device_address = espp::At581x::DEFAULT_ADDRESS,
                                            .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                                            .scl_speed_hz = i2c.config().clk_speed,
                                            .log_level = espp::Logger::Verbosity::WARN},
                                           ec);
  if (!radar_dev) {
    logger.error("Could not add AT581X I2C device: {}", ec.message());
    return;
  }

  // Create the radar driver. auto_init writes the configuration and resets the RF frontend.
  espp::At581x radar({
      .write = espp::make_i2c_addressed_write(radar_dev),
      .read_register = espp::make_i2c_addressed_read_register(radar_dev),
      .sensing_distance = g_sensing_distance.load(), // 0..1023, larger = farther / more sensitive
      .trigger_keep_time_ms = 1000,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // The AT581X reports presence on an active-high output GPIO. If it is wired up, attach an
  // interrupt to it so we get notified on presence/motion transitions.
  std::unique_ptr<espp::Interrupt> presence_interrupt;
  if (CONFIG_EXAMPLE_RADAR_OUTPUT_GPIO >= 0) {
    presence_interrupt = std::make_unique<espp::Interrupt>(espp::Interrupt::Config{
        .interrupts = {{
            .gpio_num = CONFIG_EXAMPLE_RADAR_OUTPUT_GPIO,
            .callback =
                [](const espp::Interrupt::Event &event) {
                  g_presence = event.active;
                  if (event.active) {
                    g_presence_count++;
                  }
                  logger.info("Radar presence {}", event.active ? "DETECTED" : "cleared");
                },
            .active_level = espp::Interrupt::ActiveLevel::HIGH,
            .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
            .pulldown_enabled = true,
        }},
        .task_config = {.name = "radar presence", .stack_size_bytes = 4096},
        .log_level = espp::Logger::Verbosity::WARN,
    });
    logger.info("Watching radar output on GPIO {}", CONFIG_EXAMPLE_RADAR_OUTPUT_GPIO);
  } else {
    logger.warn("No radar output GPIO configured (CONFIG_EXAMPLE_RADAR_OUTPUT_GPIO); presence "
                "interrupt disabled. Set it to receive presence events.");
  }
  //! [at581x example]

#if defined(CONFIG_EXAMPLE_HARDWARE_BOX3_SENSOR)
  // On the ESP32-S3-BOX-3 (which has a screen), show the live radar status on the display.
  static std::recursive_mutex lvgl_mutex;
  lv_obj_t *status_label = nullptr;
  lv_obj_t *detail_label = nullptr;
  std::unique_ptr<espp::Task> lv_task;

  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::WARN);
  if (box.initialize_lcd() && box.initialize_display(box.lcd_width() * 50)) {
    box.brightness(100.0f);
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);

    lv_obj_t *bg = lv_obj_create(lv_screen_active());
    lv_obj_set_size(bg, box.lcd_width(), box.lcd_height());
    lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "AT581X Radar");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    status_label = lv_label_create(lv_screen_active());
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, -10);

    detail_label = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_align(detail_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(detail_label, LV_ALIGN_BOTTOM_MID, 0, -20);

    // Run the LVGL task handler periodically to render the screen.
    lv_task = std::make_unique<espp::Task>(espp::Task::Config{
        .callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
          {
            std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
            lv_task_handler();
          }
          std::unique_lock<std::mutex> lock(m);
          cv.wait_for(lock, 16ms);
          return false; // don't stop the task
        },
        .task_config = {.name = "lvgl", .stack_size_bytes = 6 * 1024},
    });
    lv_task->start();
  } else {
    logger.error("Failed to initialize the box-3 display");
  }
#endif

  // Refresh the (optional) display and log the live radar status.
  while (true) {
#if defined(CONFIG_EXAMPLE_HARDWARE_BOX3_SENSOR)
    if (status_label) {
      bool present = g_presence.load();
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      lv_label_set_text(status_label, present ? "PRESENCE" : "no presence");
      lv_obj_set_style_text_color(
          status_label,
          present ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_GREY), 0);
      lv_obj_align(status_label, LV_ALIGN_CENTER, 0, -10);
      static std::string detail;
      detail = "detections: " + std::to_string(g_presence_count.load()) +
               "\nsensing distance: " + std::to_string(g_sensing_distance.load());
      lv_label_set_text(detail_label, detail.c_str());
      lv_obj_align(detail_label, LV_ALIGN_BOTTOM_MID, 0, -20);
    }
#endif
    logger.debug("presence={} detections={} distance={}", g_presence.load(),
                 g_presence_count.load(), g_sensing_distance.load());
    std::this_thread::sleep_for(500ms);
  }
}
