/**
 * @file smartpanle_sc01_plus_example.cpp
 * @brief Smart Panlee SC01 Plus BSP Example
 */

#include <chrono>
#include <thread>
#include <vector>

#include "smartpanle-sc01-plus.hpp"

#include "task.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
static std::vector<lv_obj_t *> circles;
static size_t next_circle_index = 0;
static std::vector<uint8_t> audio_bytes;
static std::recursive_mutex lvgl_mutex;
static lv_obj_t *background = nullptr;
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();
static void initialize_circles();
static void rotate_display();
static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::SmartPanleeSc01Plus &board);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "SC01 Plus Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");
  circles.reserve(MAX_CIRCLES);

  //! [smartpanle sc01 plus example]
  auto &board = espp::SmartPanleeSc01Plus::get();

  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }

  if (!board.initialize_display(board.display_width() * 40)) {
    logger.error("Failed to initialize display!");
    return;
  }

  auto touch_callback = [&](const auto &touch) {
    static auto previous_touchpad_data = board.touchpad_convert(touch);
    auto touchpad_data = board.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      previous_touchpad_data = touchpad_data;
      if (touchpad_data.num_touch_points > 0) {
        play_click(board);
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };

  if (!board.initialize_touch(touch_callback)) {
    logger.warn("Touch initialization did not complete cleanly");
  }

  if (!board.initialize_audio()) {
    logger.warn("Audio initialization did not complete cleanly");
  } else {
    size_t wav_size = 0;
    size_t wav_sample_rate = 0;
    if (load_audio(wav_size, wav_sample_rate)) {
      logger.info("Loaded {} bytes of audio at {} Hz", wav_size, wav_sample_rate);
      board.audio_sample_rate(wav_sample_rate);
      board.volume(30.0f);
      board.mute(false);
    } else {
      logger.warn("Could not load the embedded click sound");
    }
  }

  board.brightness(80.0f);

  if (!board.initialize_sdcard()) {
    logger.info("No microSD card mounted");
  }

  auto i2s = board.i2s_pins();
  auto rs485 = board.rs485_pins();
  logger.info("I2S pins: bclk={}, ws={}, dout={}", i2s.bclk, i2s.ws, i2s.dout);
  logger.info("RS485 pins: rts={}, rxd={}, txd={}", rs485.rts, rs485.rxd, rs485.txd);

  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, board.display_width(), board.display_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(8, 12, 24), 0);

  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Smart Panlee SC01 Plus\n\nTouch the screen to draw and play a click.\n"
                           "Press refresh to rotate.\nCheck serial output for SD card, pin, and "
                           "audio info.");
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 16, 16);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);

  lv_obj_t *btn = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn, 56, 56);
  lv_obj_align(btn, LV_ALIGN_TOP_RIGHT, -12, 12);
  lv_obj_t *btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, LV_SYMBOL_REFRESH);
  lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, 0);
  background = bg;
  initialize_circles();
  lv_obj_add_event_cb(
      btn,
      [](lv_event_t *event) {
        (void)event;
        rotate_display();
      },
      LV_EVENT_PRESSED, nullptr);

  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

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
                          .stack_size_bytes = 6 * 1024,
                      }});
  lv_task.start();
  //! [smartpanle sc01 plus example]

  while (true) {
    std::this_thread::sleep_for(1s);
    if (board.is_sd_card_available()) {
      uint32_t size_mb = 0;
      uint32_t free_mb = 0;
      if (board.get_sd_card_info(&size_mb, &free_mb)) {
        logger.info("microSD: size={} MB free={} MB", size_mb, free_mb);
      }
    }
  }
}

static void draw_circle(int x0, int y0, int radius) {
  if (circles.empty()) {
    return;
  }

  auto *circle = circles[next_circle_index];
  lv_obj_set_size(circle, radius * 2, radius * 2);
  lv_obj_align(circle, LV_ALIGN_TOP_LEFT, x0 - radius, y0 - radius);
  lv_obj_clear_flag(circle, LV_OBJ_FLAG_HIDDEN);

  next_circle_index = (next_circle_index + 1) % circles.size();
}

static void clear_circles() {
  for (auto *circle : circles) {
    lv_obj_add_flag(circle, LV_OBJ_FLAG_HIDDEN);
  }
  next_circle_index = 0;
}

static void initialize_circles() {
  if (!circles.empty()) {
    return;
  }

  circles.reserve(MAX_CIRCLES);
  for (size_t i = 0; i < MAX_CIRCLES; ++i) {
    auto *circle = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(circle);
    lv_obj_set_size(circle, 20, 20);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(circle, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_bg_opa(circle, LV_OPA_70, 0);
    lv_obj_set_style_border_width(circle, 0, 0);
    lv_obj_clear_flag(circle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(circle, LV_OBJ_FLAG_HIDDEN);
    circles.push_back(circle);
  }
}

static void rotate_display() {
  auto &board = espp::SmartPanleeSc01Plus::get();
  std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
  clear_circles();
  static auto rotation = LV_DISPLAY_ROTATION_0;
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  auto *display = lv_display_get_default();
  lv_disp_set_rotation(display, rotation);
  if (background) {
    lv_obj_set_size(background, board.rotated_display_width(), board.rotated_display_height());
  }
}

static bool load_audio(size_t &out_size, size_t &out_sample_rate) {
  if (!audio_bytes.empty()) {
    out_size = audio_bytes.size();
    return true;
  }

  extern const uint8_t click_wav_start[] asm("_binary_click_wav_start");
  extern const uint8_t click_wav_end[] asm("_binary_click_wav_end");
  audio_bytes = std::vector<uint8_t>(click_wav_start, click_wav_end);
  if (audio_bytes.size() < 44) {
    audio_bytes.clear();
    return false;
  }

  auto sample_rate = *(reinterpret_cast<const uint32_t *>(&audio_bytes[24]));
  if (audio_bytes.size() > 44) {
    audio_bytes.erase(audio_bytes.begin(), audio_bytes.begin() + 44);
  }
  out_size = audio_bytes.size();
  out_sample_rate = sample_rate;
  return true;
}

static void play_click(espp::SmartPanleeSc01Plus &board) {
  if (audio_bytes.empty()) {
    return;
  }

  auto audio_buffer_size = board.audio_buffer_size();
  if (audio_buffer_size == 0) {
    return;
  }

  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    auto bytes_to_play = std::min(audio_buffer_size, audio_bytes.size() - offset);
    board.play_audio(audio_bytes.data() + offset, static_cast<uint32_t>(bytes_to_play));
    offset += bytes_to_play;
  }
}
