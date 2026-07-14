#include <chrono>
#include <cmath>
#include <numbers>
#include <thread>
#include <vector>

#include "m5stack-cardputer.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

static constexpr auto TAG = "cardputer_example";

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

  // initialize the RGB LED and the sound subsystem (the microphone shares a
  // pin with the speaker, so this example only uses the speaker)
  if (!cardputer.initialize_led()) {
    logger.error("Failed to initialize RGB LED!");
    return;
  }
  if (!cardputer.initialize_sound()) {
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
      // toggle the IMU overlay
      bool visible = gui.toggle_imu_visible();
      gui.set_status_text(visible ? "IMU overlay shown" : "IMU overlay hidden");
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
  bool have_imu = false;
  if (cardputer.variant() == espp::M5StackCardputer::Variant::ADV) {
    have_imu = cardputer.initialize_imu();
    if (!have_imu) {
      logger.warn("Could not initialize the IMU!");
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

  // periodically update the status bar with the battery voltage / state of
  // charge, and (on the ADV) the IMU overlay with the latest accelerometer
  // and gyroscope readings
  static constexpr auto imu_period = 100ms;
  int loops_per_battery_update = std::chrono::seconds(5) / imu_period;
  int loop_count = 0;
  while (true) {
    if (have_imu && gui.imu_visible()) {
      auto imu = cardputer.imu();
      std::error_code ec;
      if (imu->update(std::chrono::duration<float>(imu_period).count(), ec)) {
        auto accel = imu->get_accelerometer();
        auto gyro = imu->get_gyroscope();
        gui.set_imu_text(fmt::format("a {:+.1f} {:+.1f} {:+.1f}\ng {:+5.0f} {:+5.0f} {:+5.0f}",
                                     accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z));
      }
    }
    if ((loop_count % loops_per_battery_update) == 0) {
      gui.set_status_text(fmt::format("Battery: {:.2f} V ({:.0f}%)", cardputer.battery_voltage(),
                                      cardputer.battery_soc()));
    }
    loop_count++;
    std::this_thread::sleep_for(imu_period);
  }
  //! [m5stack-cardputer example]
}
