#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <numbers>
#include <span>
#include <string>
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

// LoRa send state: fn+0 (F10) captures the text to send and requests a
// transmit; the main loop performs the (blocking) transmit so the keyboard
// scanner task stays responsive.
static std::atomic<bool> lora_send_requested{false};
static std::mutex lora_tx_mutex;
static std::string lora_tx_message;

// render a received LoRa payload as printable text (non-printable bytes, e.g.
// from an unrelated transmission on the same frequency, are shown as '.') so a
// stray frame cannot corrupt the on-screen log
static std::string printable(const std::vector<uint8_t> &data) {
  std::string out;
  out.reserve(data.size());
  for (uint8_t b : data) {
    out.push_back((b >= 0x20 && b < 0x7f) ? static_cast<char>(b) : '.');
  }
  return out;
}

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

  // whether the board has a working IMU / microphone / LoRa radio / GPS; set
  // after keyboard / variant detection below, referenced by the keypress
  // callback
  static bool have_imu = false;
  static bool have_mic = false;
  static bool have_lora = false;
  static bool have_gps = false;
  static std::shared_ptr<espp::Sx126x> lora_radio;

  // The text being composed lives in the (hidden-when-not-active) Text tab, so
  // on the LoRa tab mirror it into the status bar as it is typed - otherwise
  // you cannot see what you are about to send.
  auto show_lora_compose = [&]() {
    std::string composed = gui.get_text();
    gui.set_status_text(composed.empty() ? "Type a message; fn+0 sends" : ("> " + composed));
  };

  // the keyboard scanner delivers one event per key state change; use it to
  // drive the text editor, play key-click sounds, and show what's happening
  // in the status bar
  auto keypress_callback = [&](const espp::M5StackCardputer::KeyEvent &event) {
    if (!event.pressed) {
      return;
    }
    // fn+Tab cycles through the tabs. fn does not change a key's character
    // value, so fn+Tab arrives as a Tab press ('\t') with the fn modifier held.
    if (event.modifiers.fn && event.value == '\t') {
      Gui::Tab tab = gui.next_tab();
      if (tab == Gui::Tab::LORA && have_lora) {
        show_lora_compose();
      }
      play_beep(cardputer, 660.0f);
      return;
    }
    if (event.special == espp::M5StackCardputer::SpecialKey::F1) {
      // jump to the Help tab
      gui.select_tab(Gui::Tab::HELP);
      gui.set_status_text("Help tab");
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F2) {
      // jump to the IMU tab
      gui.select_tab(Gui::Tab::IMU);
      gui.set_status_text(have_imu ? "IMU tab" : "No IMU on this board");
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
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F5 ||
               event.special == espp::M5StackCardputer::SpecialKey::F6) {
      // speaker volume down / up (the beep gives immediate feedback)
      float delta = event.special == espp::M5StackCardputer::SpecialKey::F5 ? -10.0f : 10.0f;
      cardputer.volume(cardputer.volume() + delta);
      gui.set_status_text(fmt::format("Speaker volume: {:.0f}%", cardputer.volume()));
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F7 ||
               event.special == espp::M5StackCardputer::SpecialKey::F8) {
      // microphone volume down / up (heard on the next recording)
      float delta = event.special == espp::M5StackCardputer::SpecialKey::F7 ? -5.0f : 5.0f;
      cardputer.microphone_volume(cardputer.microphone_volume() + delta);
      gui.set_status_text(
          fmt::format("Mic volume: {:.0f}% (75% = 0 dB)", cardputer.microphone_volume()));
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F9) {
      // jump to the LoRa tab
      gui.select_tab(Gui::Tab::LORA);
      if (have_lora) {
        show_lora_compose();
      } else {
        gui.set_status_text("No LoRa module (attach the Cap)");
      }
      play_beep(cardputer, 660.0f);
    } else if (event.special == espp::M5StackCardputer::SpecialKey::F10) {
      // send the current text area contents over LoRa (the main loop performs
      // the actual, blocking transmit)
      if (!have_lora) {
        gui.set_status_text("No LoRa module (attach the Cap)");
      } else {
        std::string text = gui.get_text();
        if (text.empty()) {
          gui.set_status_text("Type a message first, then fn+0");
        } else {
          {
            std::lock_guard<std::mutex> lk(lora_tx_mutex);
            lora_tx_message = text;
          }
          lora_send_requested = true;
          gui.clear_text();
          gui.select_tab(Gui::Tab::LORA);
          gui.set_status_text("Sending over LoRa...");
        }
      }
      play_beep(cardputer, 660.0f);
    } else if (event.special != espp::M5StackCardputer::SpecialKey::NONE) {
      gui.handle_special_key(event.special);
      gui.set_status_text(espp::M5StackCardputer::special_key_name(event.special));
      play_beep(cardputer, 660.0f);
    } else if (event.value != 0) {
      gui.add_char(event.value);
      // on the LoRa tab, mirror the composed text into the status bar so it is
      // visible as you type (the text box itself is on the Text tab)
      if (gui.active_tab() == Gui::Tab::LORA && have_lora) {
        show_lora_compose();
      }
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
    if (!have_imu) {
      logger.warn("Could not initialize the IMU!");
      gui.set_imu_text("IMU init failed");
    }
  } else {
    gui.set_imu_text("No IMU on this board\n(Cardputer ADV only)");
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

  // Initialize the LoRa radio on the LoRa+GPS Cap (SX1262) and wire it to the
  // LoRa tab. This uses the same radio settings as the T-Deck example (US
  // LongFast modulation, private sync word 0x12), so a Cardputer and a T-Deck
  // each running their example will exchange text messages. The Cap is a
  // Cardputer ADV accessory; on a board without it, initialization fails and
  // the LoRa tab reports that it is unavailable.
  espp::Sx126x::RadioConfig lora_config{};
  lora_config.sync_word = 0x12; // private link, matches the t-deck example
  have_lora = cardputer.initialize_lora(lora_config);
  if (have_lora) {
    lora_radio = cardputer.lora();
    // deliver received packets to the LoRa tab (runs in the BSP interrupt task)
    lora_radio->set_receive_callback([&](const espp::Sx126x::RxPacket &packet) {
      gui.add_lora_message(
          fmt::format("RX {:.0f}dBm: {}", packet.status.rssi, printable(packet.data)));
    });
    std::error_code ec;
    if (lora_radio->start_receive(ec)) {
      gui.set_lora_status(fmt::format("Listening @ {:.3f} MHz, SF11/BW250 (fn+0 sends)",
                                      lora_radio->radio_config().frequency_hz / 1e6f));
    } else {
      logger.error("Failed to start LoRa receive: {}", ec.message());
      gui.set_lora_status(fmt::format("LoRa RX failed: {}", ec.message()));
      have_lora = false;
    }
  } else {
    logger.warn("Could not initialize LoRa (is the LoRa+GPS Cap attached? ADV only)");
    gui.set_lora_status("LoRa unavailable (attach the Cap)");
  }

  // Initialize the GNSS receiver on the same LoRa+GPS Cap (ATGM336H, 115200
  // baud) and show the fix on the GPS tab. GPS and LoRa are both on the Cap,
  // so use the LoRa result as the "Cap present" signal; the fix callback runs
  // on the GPS reader task (its GUI calls are thread-safe).
  if (have_lora) {
    gui.set_gps_text("GPS: acquiring fix...\n(needs a clear sky view)");
    have_gps = cardputer.initialize_gps([&](const espp::GpsFix &fix) {
      if (fix.valid) {
        gui.set_gps_text(fmt::format("Fix: {} sats  HDOP {:.1f}\n{:.5f}, {:.5f}\nAlt {:.0f} m\n"
                                     "{:02d}:{:02d}:{:04.1f} UTC\n{:.1f} kn  {:.0f} deg",
                                     (int)fix.num_satellites, fix.hdop, fix.latitude, fix.longitude,
                                     fix.altitude, (int)fix.hour, (int)fix.minute, fix.second,
                                     fix.speed_knots, fix.course_degrees));
      } else {
        gui.set_gps_text(fmt::format("Acquiring fix...\n{} sats in view\n(needs a clear sky view)",
                                     (int)fix.num_satellites));
      }
    });
    if (!have_gps) {
      logger.warn("Could not initialize the GPS!");
      gui.set_gps_text("GPS init failed (see log)");
    }
  } else {
    gui.set_gps_text("GPS unavailable\n(attach the LoRa+GPS Cap)");
  }

  // Main loop: service LoRa sends, stream any active playback to the speaker,
  // update the IMU / Sys tabs, and periodically show the battery voltage /
  // state of charge in the status bar
  static constexpr auto loop_period = 50ms;
  const int loops_per_imu_update = 2;                                     // 100 ms
  const int loops_per_sys_update = std::chrono::seconds(1) / loop_period; // 1 s
  const int loops_per_battery_update = std::chrono::seconds(5) / loop_period;
  size_t play_offset = 0;
  bool was_recording = false;
  int loop_count = 0;
  while (true) {
    // service a LoRa send requested from the keyboard (fn+0). transmit()
    // blocks for the packet's time-on-air (a few hundred ms at SF11) then
    // returns the radio to receive, so doing it here keeps the keyboard and
    // LVGL tasks responsive.
    if (have_lora && lora_send_requested.exchange(false)) {
      std::string msg;
      {
        std::lock_guard<std::mutex> lk(lora_tx_mutex);
        msg = lora_tx_message;
      }
      std::span<const uint8_t> payload{reinterpret_cast<const uint8_t *>(msg.data()), msg.size()};
      std::error_code ec;
      if (lora_radio->transmit(payload, 3s, ec)) {
        gui.add_lora_message("TX: " + msg);
        gui.set_status_text("Sent over LoRa");
        logger.info("LoRa sent: {}", msg);
      } else {
        gui.add_lora_message(fmt::format("TX failed: {}", ec.message()));
        gui.set_status_text("LoRa send failed");
        logger.error("LoRa transmit failed: {}", ec.message());
      }
    }
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

    // update the IMU tab only while it is the active tab (the readings are
    // only visible there)
    if (have_imu && gui.active_tab() == Gui::Tab::IMU && (loop_count % loops_per_imu_update) == 0) {
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
    // keep the Sys tab's board info current
    if ((loop_count % loops_per_sys_update) == 0) {
      gui.set_system_text(fmt::format(
          "Board: {}\nBattery: {:.2f} V ({:.0f}%)\nSpeaker {:.0f}%  Mic {:.0f}%\nAudio {} Hz\n"
          "LoRa {}  GPS {}",
          espp::M5StackCardputer::variant_name(cardputer.variant()), cardputer.battery_voltage(),
          cardputer.battery_soc(), cardputer.volume(), cardputer.microphone_volume(),
          cardputer.audio_sample_rate(), have_lora ? "on" : "off", have_gps ? "on" : "off"));
    }
    // Periodically show the battery in the status bar - but not on the LoRa
    // tab, where the status bar mirrors the message being composed (see the
    // keypress callback), nor while recording / playing.
    if ((loop_count % loops_per_battery_update) == 0 && !recording && !playing &&
        gui.active_tab() != Gui::Tab::LORA) {
      gui.set_status_text(fmt::format("Battery: {:.2f} V ({:.0f}%)", cardputer.battery_voltage(),
                                      cardputer.battery_soc()));
    }
    loop_count++;
    std::this_thread::sleep_for(loop_period);
  }
  //! [m5stack-cardputer example]
}
