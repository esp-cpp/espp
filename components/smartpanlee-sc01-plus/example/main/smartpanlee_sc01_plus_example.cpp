/**
 * @file smartpanlee_sc01_plus_example.cpp
 * @brief Smart Panlee SC01 Plus BSP Example
 */

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include "smartpanlee-sc01-plus.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

static std::vector<uint8_t> audio_bytes;

static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::SmartPanleeSc01Plus &board);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "SC01 Plus Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [smartpanlee sc01 plus example]
  auto &board = espp::SmartPanleeSc01Plus::get();

  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }

  if (!board.initialize_display(board.display_width() * 40)) {
    logger.error("Failed to initialize display!");
    return;
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

  // create the GUI: builds the UI (info label, buttons, circle layer) and
  // starts the task which updates LVGL. All of its public methods are
  // thread-safe, so the touch callback below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Smart Panlee SC01 Plus\n\nTouch the screen to draw and play a "
                     "click.\nPress " LV_SYMBOL_REFRESH " to rotate.\nPress " LV_SYMBOL_TRASH
                     " to clear.\nCheck serial output for SD card, pin, and audio info.");

  // initialize the touchpad after the GUI exists so touch events can update
  // it immediately; each touch draws a circle and plays a click sound
  auto touch_callback = [&](const auto &touch) {
    static auto previous_touchpad_data = board.touchpad_convert(touch);
    auto touchpad_data = board.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      previous_touchpad_data = touchpad_data;
      if (touchpad_data.num_touch_points > 0) {
        play_click(board);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  if (!board.initialize_touch(touch_callback)) {
    logger.warn("Touch initialization did not complete cleanly");
  }
  //! [smartpanlee sc01 plus example]

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
