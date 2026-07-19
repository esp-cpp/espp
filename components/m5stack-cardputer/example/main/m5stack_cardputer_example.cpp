#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <numbers>
#include <thread>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "m5stack-cardputer.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

static constexpr auto TAG = "cardputer_example";

// Voice-quality sample rate for the speaker + microphone. On the ADV the two
// run full duplex and share this rate; keeping it low keeps the recording
// buffer small (neither Cardputer variant has PSRAM, so the recording
// usually lives in internal RAM).
static constexpr uint32_t AUDIO_SAMPLE_RATE_HZ = 16000;

// Audio recording state (written by the microphone callback, read/controlled
// from the keyboard callback and main loop)
static constexpr size_t MAX_RECORDING_SECONDS = 30;     // when PSRAM is available
static constexpr size_t FALLBACK_RECORDING_SECONDS = 3; // internal RAM fallback
static uint8_t *recording_buffer = nullptr;
static size_t recording_capacity = 0;
static std::atomic<bool> recording{false};
static std::atomic<size_t> recording_len{0};
static std::atomic<bool> playing{false};
// wall-clock bounds of the capture, for reporting the measured effective
// sample rate (ordering is provided by the `recording` atomic)
static int64_t recording_start_us = 0;
static int64_t recording_last_us = 0;

// Synthesize a short fading sine beep and queue it on the speaker
static void play_beep(espp::M5StackCardputer &cardputer, float frequency_hz) {
  static constexpr float duration_s = 0.05f;
  const uint32_t sample_rate = cardputer.audio_sample_rate();
  const size_t num_samples = static_cast<size_t>(duration_s * sample_rate);
  std::vector<uint8_t> data(num_samples * sizeof(int16_t));
  auto *samples = reinterpret_cast<int16_t *>(data.data());
  for (size_t i = 0; i < num_samples; i++) {
    float t = static_cast<float>(i) / sample_rate;
    // fade the beep out over its duration to avoid a click at the end
    float envelope = 1.0f - (static_cast<float>(i) / num_samples);
    samples[i] =
        static_cast<int16_t>(INT16_MAX * 0.5f * envelope *
                             std::sin(2.0f * std::numbers::pi_v<float> * frequency_hz * t));
  }
  cardputer.play_audio(data);
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = TAG, .level = espp::Logger::Verbosity::INFO});

  //! [m5stack-cardputer example]
  espp::M5StackCardputer &cardputer = espp::M5StackCardputer::get();
  cardputer.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LCD
  if (!cardputer.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // initialize the display, using a pixel buffer of 50 lines
  static constexpr size_t pixel_buffer_size = cardputer.lcd_width() * 50;
  if (!cardputer.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the RGB LED and the sound subsystem
  if (!cardputer.initialize_led()) {
    logger.error("Failed to initialize RGB LED!");
    return;
  }
  if (!cardputer.initialize_sound(AUDIO_SAMPLE_RATE_HZ)) {
    logger.error("Failed to initialize sound!");
    return;
  }
  cardputer.volume(60.0f);

  // try to mount the uSD card (warn and continue if it's not inserted)
  if (!cardputer.initialize_sdcard({})) {
    logger.warn("Could not mount the uSD card; is one inserted?");
  }

  // create the GUI (a small keyboard-driven text editor)
  static Gui gui({});

  // print the controls (also available on-screen via the fn+1 help popup)
  logger.info("Controls:\n{}", Gui::HELP_TEXT);

  // whether the board has a working IMU / microphone; set after keyboard /
  // variant detection below, referenced by the keypress callback
  static bool have_imu = false;
  static bool have_mic = false;

  // the keyboard scanner delivers one event per key state change; use it to
  // drive the text editor, play key-click sounds, and show what's happening
  // in the status bar
  auto keypress_callback = [&](const espp::M5StackCardputer::KeyEvent &event) {
    if (!event.pressed) {
      return;
    }
    if (event.special == espp::M5StackCardputer::SpecialKey::F1) {
      // toggle the help popup
      bool shown = gui.toggle_help();
      gui.set_status_text(shown ? "Help (fn+1 to close)" : "Ready");
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F2) {
      // toggle the IMU popup
      if (have_imu) {
        bool visible = gui.toggle_imu_visible();
        gui.set_status_text(visible ? "IMU popup shown" : "IMU popup hidden");
      } else {
        gui.set_status_text("No IMU on this board");
      }
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F3) {
      // start / stop recording from the microphone
      if (!have_mic || recording_capacity == 0) {
        gui.set_status_text("No mic recording on this board");
      } else if (recording) {
        recording = false;
        gui.set_status_text(
            fmt::format("Recorded {:.1f}s (fn+4 plays)",
                        static_cast<float>(recording_len) /
                            (cardputer.microphone_sample_rate() * sizeof(int16_t))));
      } else {
        playing = false;
        recording_len = 0;
        recording_start_us = esp_timer_get_time();
        recording_last_us = recording_start_us;
        recording = true;
        gui.set_status_text("Recording... (fn+3 stops)");
      }
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F4) {
      // play back the recording (the main loop streams it to the speaker)
      if (recording_len == 0) {
        gui.set_status_text("Nothing recorded yet (fn+3 records)");
      } else if (playing) {
        playing = false;
        gui.set_status_text("Playback stopped");
      } else {
        recording = false;
        playing = true;
        gui.set_status_text("Playing... (fn+4 stops)");
      }
      play_beep(cardputer, 660.0f);
    } else if (event.special != espp::M5StackCardputer::SpecialKey::NONE) {
      gui.handle_special_key(event.special);
      gui.set_status_text(espp::M5StackCardputer::special_key_name(event.special));
      play_beep(cardputer, 660.0f);
    } else if (event.value != 0) {
      gui.add_char(event.value);
      play_beep(cardputer, 880.0f);
    } else {
      // a modifier key by itself
      std::string status;
      if (event.modifiers.fn)
        status += "fn ";
      if (event.modifiers.shift)
        status += "shift ";
      if (event.modifiers.ctrl)
        status += "ctrl ";
      if (event.modifiers.opt)
        status += "opt ";
      if (event.modifiers.alt)
        status += "alt ";
      gui.set_status_text(status.empty() ? "Ready" : status);
    }
  };
  // the keyboard scanner auto-detects the board variant: the original's
  // 74HC138 GPIO matrix or the ADV's TCA8418 I2C keyboard controller
  if (!cardputer.initialize_keyboard(keypress_callback)) {
    logger.error("Failed to initialize keyboard!");
    return;
  }
  logger.info("Board variant: {}", espp::M5StackCardputer::variant_name(cardputer.variant()));

  // the ADV has a BMI270 IMU on the internal I2C bus; initialize it if we're
  // on one (warn and continue otherwise - the original has no IMU)
  if (cardputer.variant() == espp::M5StackCardputer::Variant::ADV) {
    have_imu = cardputer.initialize_imu();
    if (have_imu) {
      // show the IMU popup by default (fn+2 toggles it)
      gui.toggle_imu_visible();
    } else {
      logger.warn("Could not initialize the IMU!");
    }
  }

  // On the ADV the ES8311 codec runs full duplex, so the microphone can be
  // used at the same time as the speaker (it shares the speaker's sample
  // rate). The original cannot: its PDM microphone clock and the speaker
  // word-select share GPIO 43, and this example uses the speaker.
  if (cardputer.variant() == espp::M5StackCardputer::Variant::ADV) {
    auto mic_callback = [](const uint8_t *data, size_t num_bytes) {
      if (!recording) {
        return;
      }
      size_t offset = recording_len;
      size_t to_copy = std::min(num_bytes, recording_capacity - offset);
      if (to_copy > 0) {
        memcpy(recording_buffer + offset, data, to_copy);
        recording_last_us = esp_timer_get_time();
        recording_len = offset + to_copy;
      }
      if (recording_len >= recording_capacity) {
        // buffer full; the main loop notices recording went false and
        // updates the status bar
        recording = false;
      }
    };
    have_mic = cardputer.initialize_microphone(mic_callback);
    if (have_mic) {
      // allocate the recording buffer: prefer PSRAM (neither Cardputer
      // variant ships with it, but boards / mods that have it get a much
      // longer recording), fall back to a few seconds in internal RAM
      size_t bytes_per_second = cardputer.microphone_sample_rate() * sizeof(int16_t);
      recording_capacity = MAX_RECORDING_SECONDS * bytes_per_second;
      recording_buffer = static_cast<uint8_t *>(
          heap_caps_malloc(recording_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (recording_buffer == nullptr) {
        recording_capacity = FALLBACK_RECORDING_SECONDS * bytes_per_second;
        recording_buffer =
            static_cast<uint8_t *>(heap_caps_malloc(recording_capacity, MALLOC_CAP_8BIT));
      }
      if (recording_buffer == nullptr) {
        logger.warn("Could not allocate a recording buffer; recording disabled");
        recording_capacity = 0;
      } else {
        logger.info("Recording buffer: {} KB ({} s at {} Hz)", recording_capacity / 1024,
                    recording_capacity / bytes_per_second, cardputer.microphone_sample_rate());
      }
    } else {
      logger.warn("Could not initialize the microphone!");
    }
  }

  // the G0 (BOOT) button cycles the RGB LED color
  static std::atomic<int> led_hue{0};
  auto button_callback = [&](const espp::Interrupt::Event &event) {
    if (event.active) {
      led_hue = (led_hue + 60) % 360;
      cardputer.led(espp::Hsv(static_cast<float>(led_hue), 1.0f, 0.2f));
      play_beep(cardputer, 440.0f);
    }
  };
  if (!cardputer.initialize_button(button_callback)) {
    logger.error("Failed to initialize button!");
    return;
  }

  // set the initial LED color
  cardputer.led(espp::Hsv(static_cast<float>(led_hue), 1.0f, 0.2f));

  // Main loop: stream any active playback to the speaker, update the IMU
  // popup, and periodically show the battery voltage / state of charge in
  // the status bar
  static constexpr auto loop_period = 50ms;
  const int loops_per_imu_update = 2; // 100 ms
  const int loops_per_battery_update = std::chrono::seconds(5) / loop_period;
  size_t play_offset = 0;
  bool was_recording = false;
  int loop_count = 0;
  while (true) {
    // feed the active playback in chunks, advancing by however much the
    // speaker's stream buffer accepted
    if (playing) {
      size_t len = recording_len;
      if (play_offset >= len) {
        playing = false;
        play_offset = 0;
        gui.set_status_text("Playback done");
      } else {
        play_offset += cardputer.play_audio(recording_buffer + play_offset,
                                            std::min<size_t>(len - play_offset, 4096));
      }
    } else {
      play_offset = 0;
    }
    // notice when the recording stopped (buffer filled up, or fn+3 / fn+4)
    bool now_recording = recording;
    if (was_recording && !now_recording) {
      // report the measured capture rate: samples recorded over the wall
      // clock they took to arrive should match the nominal sample rate
      size_t num_samples = recording_len / sizeof(int16_t);
      float elapsed_s = static_cast<float>(recording_last_us - recording_start_us) / 1e6f;
      float effective_hz = elapsed_s > 0.0f ? num_samples / elapsed_s : 0.0f;
      logger.info("Recorded {} samples in {:.2f} s (~{:.0f} Hz effective, {} Hz nominal)",
                  num_samples, elapsed_s, effective_hz, cardputer.microphone_sample_rate());
      gui.set_status_text(fmt::format("Recorded {:.1f}s (fn+4 plays)",
                                      static_cast<float>(recording_len) /
                                          (cardputer.microphone_sample_rate() * sizeof(int16_t))));
    }
    was_recording = now_recording;

    if (have_imu && gui.imu_visible() && (loop_count % loops_per_imu_update) == 0) {
      auto imu = cardputer.imu();
      std::error_code ec;
      if (imu->update(std::chrono::duration<float>(loop_period * loops_per_imu_update).count(),
                      ec)) {
        auto accel = imu->get_accelerometer();
        auto gyro = imu->get_gyroscope();
        gui.set_imu_text(fmt::format("a {:+.1f} {:+.1f} {:+.1f}\ng {:+5.0f} {:+5.0f} {:+5.0f}",
                                     accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z));
      }
    }
    // don't overwrite the recording / playback status with the battery
    if ((loop_count % loops_per_battery_update) == 0 && !recording && !playing) {
      gui.set_status_text(fmt::format("Battery: {:.2f} V ({:.0f}%)", cardputer.battery_voltage(),
                                      cardputer.battery_soc()));
    }
    loop_count++;
    std::this_thread::sleep_for(loop_period);
  }
  //! [m5stack-cardputer example]
}
