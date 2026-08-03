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
  gui.set_label_text(
      fmt::format("Smart Panlee SC01 Plus\n\nTouch the screen to draw and play a click.\nPress "
                  "{} to rotate.\nPress {} to clear.\nThe Audio tab plays the click sound and "
                  "adjusts / mutes the volume.\nCheck serial output for SD card, pin, and audio "
                  "info.",
                  LV_SYMBOL_REFRESH, LV_SYMBOL_TRASH));

  // the play button on the audio row plays the same click sound as a touch
  gui.set_play_callback([&]() { play_click(board); });

  // initialize the touchpad after the GUI exists so touch events can update
  // it immediately; each touch draws a circle and plays a click sound
  auto touch_callback = [&](const auto &touch) {
    static auto previous_touchpad_data = board.touchpad_convert(touch);
    auto touchpad_data = board.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      previous_touchpad_data = touchpad_data;
      if (touchpad_data.num_touch_points > 0 && gui.draw_page_active()) {
        play_click(board);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  // NOTE: this example raises the BSP interrupt-task stack size via
  // sdkconfig.defaults (CONFIG_SMARTPANLEE_SC01_PLUS_INTERRUPT_STACK_SIZE=8192);
  // the touch controller is read from that task and its error-logging path
  // needs more than the 4 KB BSP default. See the example README.
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

  // Advance by however many bytes play_audio() actually queued (it enqueues
  // only whole frames), not the requested size, so no samples are skipped. Stop
  // as soon as the stream buffer is full rather than waiting for it to drain -
  // this runs in the touch callback, so blocking would freeze the touch task
  // for the whole click. The click comfortably fits in the stream buffer.
  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    auto chunk = std::min(audio_buffer_size, audio_bytes.size() - offset);
    size_t queued = board.play_audio(audio_bytes.data() + offset, static_cast<uint32_t>(chunk));
    offset += queued;
    if (queued < chunk) {
      break; // stream buffer full for now; do not block the caller
    }
  }
}
