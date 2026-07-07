#include <chrono>
#include <stdlib.h>
#include <vector>

#include "t-deck.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

static std::vector<uint8_t> audio_bytes;

static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::TDeck &tdeck);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "T-Deck Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [t-deck example]
  espp::TDeck &tdeck = espp::TDeck::get();
  tdeck.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the uSD card
  using SdCardConfig = espp::TDeck::SdCardConfig;
  SdCardConfig sdcard_config{};
  if (!tdeck.initialize_sdcard(sdcard_config)) {
    logger.warn("Failed to initialize uSD card, there may not be a uSD card inserted!");
  }
  // initialize the sound
  if (!tdeck.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  // initialize the LCD
  if (!tdeck.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = tdeck.lcd_width() * 50;
  // initialize the LVGL display for the T-Deck
  if (!tdeck.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // create the GUI: builds the UI (label, buttons, circle layer) and starts
  // the task which updates LVGL. All of its public methods are thread-safe,
  // so the keyboard and touch callbacks below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Touch the screen!\nPress the delete key or the " LV_SYMBOL_TRASH
                     " button to clear circles.\nPress the space key or the " LV_SYMBOL_REFRESH
                     " button to rotate the display.");

  // initialize the Keyboard after the Gui exists so key presses can act on it
  // immediately
  auto keypress_callback = [&](uint8_t key) {
    logger.info("Key pressed: {}", key);
    if (key == 8) {
      // delete key will clear the circles
      logger.info("Clearing circles");
      gui.clear_circles();
    } else if (key == ' ') {
      // space key will rotate the display
      logger.info("Rotating display");
      gui.next_rotation();
    } else if (key == 'm') {
      // 'm' key will toggle audio mute
      logger.info("Toggling mute");
      tdeck.mute(!tdeck.is_muted());
      logger.info("Muted: {}", tdeck.is_muted());
    } else if (key == 'n') {
      // 'n' key will decrease audio volume (left of 'm' key)
      logger.info("Decreasing volume");
      tdeck.volume(tdeck.volume() - 10.0f);
      logger.info("Volume: {}", tdeck.volume());
    } else if (key == '$') {
      // '$' key will increase audio volume (right of 'm' key)
      logger.info("Increasing volume");
      tdeck.volume(tdeck.volume() + 10.0f);
      logger.info("Volume: {}", tdeck.volume());
    }
  };
  bool start_task = true;
  if (!tdeck.initialize_keyboard(start_task, keypress_callback)) {
    logger.error("Failed to initialize Keyboard!");
    return;
  }

  // initialize the trackball
  auto trackball_callback = [&](const auto &trackball) {
    logger.debug("Trackball: {}", trackball);
  };
  if (!tdeck.initialize_trackball(trackball_callback)) {
    logger.error("Failed to initialize trackball!");
    return;
  }

  // initialize the touchpad; each touch draws a circle (and plays a click
  // sound)
  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = tdeck.touchpad_convert(touch);
    auto touchpad_data = tdeck.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if there is a touch point, draw a circle and play a click sound
      if (touchpad_data.num_touch_points > 0) {
        play_click(tdeck);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  if (!tdeck.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // load the audio file (wav file bundled in memory)
  size_t wav_size = 0;
  size_t wav_sample_rate = 0;
  if (!load_audio(wav_size, wav_sample_rate)) {
    logger.error("Failed to load audio file!");
    return;
  }
  logger.info("Loaded {} bytes of audio", wav_size);

  logger.info("Setting audio sample rate to {} Hz", wav_sample_rate);
  tdeck.audio_sample_rate(wav_sample_rate);

  // unmute the audio and set the volume to 20%
  tdeck.mute(false);
  tdeck.volume(20.0f);

  // set the display brightness to be 75%
  tdeck.brightness(75.0f);

  // now just loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [t-deck example]
}

static bool load_audio(size_t &out_size, size_t &out_sample_rate) {
  // if the audio_bytes vector is already populated, return the size
  if (audio_bytes.size() > 0) {
    return true;
  }

  // load the audio data. these are configured in the CMakeLists.txt file

  extern const uint8_t click_wav_start[] asm("_binary_click_wav_start");
  extern const uint8_t click_wav_end[] asm("_binary_click_wav_end");
  audio_bytes = std::vector<uint8_t>(click_wav_start, click_wav_end);
  // ensure we have at least a wav header
  if (audio_bytes.size() < 44) {
    audio_bytes.clear();
    return false;
  }
  // get the sample rate from the wav header (bytes 24-27)
  uint32_t sample_rate = *(reinterpret_cast<const uint32_t *>(&audio_bytes[24]));
  // set the audio sample rate accordingly
  // decode the wav file header (first 44 bytes) and remove it
  if (audio_bytes.size() > 44) {
    audio_bytes.erase(audio_bytes.begin(), audio_bytes.begin() + 44);
  }
  out_size = audio_bytes.size();
  out_sample_rate = sample_rate;
  return true;
}

static void play_click(espp::TDeck &tdeck) {
  // use the tdeck.play_audio() function to play a sound, breaking it into
  // audio_buffer_size chunks
  auto audio_buffer_size = tdeck.audio_buffer_size();
  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    size_t bytes_to_play = std::min(audio_buffer_size, audio_bytes.size() - offset);
    tdeck.play_audio(audio_bytes.data() + offset, bytes_to_play);
    offset += bytes_to_play;
  }
}
