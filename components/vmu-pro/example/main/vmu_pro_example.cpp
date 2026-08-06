#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>

#include "vmu-pro.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

static void play_beep(espp::VmuPro &vmu, float frequency_hz);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "VMU Pro Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [vmu-pro example]
  espp::VmuPro &vmu = espp::VmuPro::get();
  vmu.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LCD
  if (!vmu.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = vmu.lcd_width() * 50;
  // initialize the LVGL display
  if (!vmu.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the sound (mono speaker via I2S amplifier)
  if (!vmu.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  vmu.volume(50.0f);

  // initialize the uSD card; warn and continue if there is no card inserted
  if (!vmu.initialize_sdcard({})) {
    logger.warn("Failed to initialize uSD card, there may not be a uSD card inserted!");
  }

  // create the GUI: builds the UI (label, cursor, circle layer) and starts
  // the task which updates LVGL. All of its public methods are thread-safe,
  // so the button callback below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  static const std::string instructions = "D-pad: move cursor\nA: draw circle\nB: clear "
                                          "circles\nMode: rotate display\nBottom: cycle "
                                          "brightness\nPower: toggle mute";
  gui.set_label_text(instructions);

  // initialize the buttons; the callback runs in the interrupt task, and the
  // Gui methods are thread-safe, so we can drive the UI directly from here
  using Button = espp::VmuPro::Button;
  static constexpr int cursor_step = 10;
  auto button_callback = [&](Button button, bool pressed) {
    logger.info("Button {}: {}", espp::VmuPro::button_name(button),
                pressed ? "pressed" : "released");
    gui.set_label_text(fmt::format("{}\n\nButton {}: {}", instructions,
                                   espp::VmuPro::button_name(button),
                                   pressed ? "pressed" : "released"));
    if (!pressed) {
      return;
    }
    switch (button) {
    case Button::DPAD_UP:
      gui.move_cursor(0, -cursor_step);
      break;
    case Button::DPAD_DOWN:
      gui.move_cursor(0, cursor_step);
      break;
    case Button::DPAD_LEFT:
      gui.move_cursor(-cursor_step, 0);
      break;
    case Button::DPAD_RIGHT:
      gui.move_cursor(cursor_step, 0);
      break;
    case Button::A:
      gui.draw_at_cursor();
      play_beep(vmu, 880.0f);
      break;
    case Button::B:
      gui.clear_circles();
      play_beep(vmu, 440.0f);
      break;
    case Button::MODE:
      gui.next_rotation();
      break;
    case Button::BOTTOM:
      gui.cycle_brightness();
      break;
    case Button::POWER:
      vmu.mute(!vmu.is_muted());
      logger.info("Audio {}", vmu.is_muted() ? "muted" : "unmuted");
      break;
    default:
      break;
    }
  };
  if (!vmu.initialize_buttons(button_callback)) {
    logger.error("Failed to initialize buttons!");
    return;
  }

  // set the display brightness to be 75%
  vmu.brightness(75.0f);

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [vmu-pro example]
}

// Synthesize a short sine-wave beep and queue it for playback. The audio
// subsystem expects 16-bit signed mono samples at the configured sample rate.
static void play_beep(espp::VmuPro &vmu, float frequency_hz) {
  static constexpr float duration_s = 0.05f;
  static constexpr float amplitude = 0.5f;
  uint32_t sample_rate = vmu.audio_sample_rate();
  size_t num_samples = static_cast<size_t>(duration_s * sample_rate);
  std::vector<int16_t> samples(num_samples);
  for (size_t i = 0; i < num_samples; i++) {
    float t = static_cast<float>(i) / sample_rate;
    // fade the beep out over its duration to avoid a click at the end
    float envelope = 1.0f - (static_cast<float>(i) / num_samples);
    samples[i] =
        static_cast<int16_t>(amplitude * envelope * 32767.0f *
                             std::sin(2.0f * std::numbers::pi_v<float> * frequency_hz * t));
  }
  vmu.play_audio(reinterpret_cast<const uint8_t *>(samples.data()),
                 samples.size() * sizeof(int16_t));
}
