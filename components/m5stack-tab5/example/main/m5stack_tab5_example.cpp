/**
 * @file m5stack_tab5_example.cpp
 * @brief M5Stack Tab5 BSP Example
 *
 * This example demonstrates the comprehensive functionality of the M5Stack Tab5
 * development board including display, touch, audio, camera, IMU, power management,
 * and communication interfaces.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "m5stack-tab5.hpp"

#include "gui.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

static std::vector<uint8_t> audio_bytes;

static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::M5StackTab5 &tab5);

// Audio recording state (written by the recording callback, read/controlled
// from the GUI button callbacks and main loop). The recorded data is 16-bit
// interleaved stereo at the current audio sample rate.
static constexpr size_t MAX_RECORDING_SECONDS = 30;     // when PSRAM is available
static constexpr size_t FALLBACK_RECORDING_SECONDS = 2; // internal RAM fallback
static uint8_t *recording_buffer = nullptr;
static size_t recording_capacity = 0;
static std::atomic<bool> recording{false};
static std::atomic<size_t> recording_len{0};
static std::atomic<bool> playing{false};
// wall-clock bounds of the capture, for reporting the measured effective
// sample rate (ordering is provided by the `recording` atomic)
static std::atomic<int64_t> recording_start_us{0};
static std::atomic<int64_t> recording_last_us{0};

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "M5Stack Tab5 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [m5stack tab5 example]
  espp::M5StackTab5 &tab5 = espp::M5StackTab5::get();
  // tab5.set_log_level(espp::Logger::Verbosity::DEBUG);
  logger.info("Running on M5Stack Tab5");

  // first let's get the internal i2c bus and probe for all devices on the bus
  logger.info("Probing internal I2C bus...");
  auto &i2c = tab5.internal_i2c();
  std::vector<uint8_t> found_addresses;
  for (uint8_t address = 1; address < 128; address++) {
    if (i2c.probe_device(address)) {
      found_addresses.push_back(address);
    }
  }
  logger.info("Found devices at addresses: {::#02x}", found_addresses);

  // Initialize the IO expanders
  logger.info("Initializing IO expanders...");
  if (!tab5.initialize_io_expanders()) {
    logger.error("Failed to initialize IO expanders!");
    return;
  }

  logger.info("Initializing lcd...");
  // initialize the LCD
  if (!tab5.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }

  // initialize the display with a pixel buffer (Tab5 is 1280x720 with 2 bytes per pixel)
  logger.info("Initializing display...");
  auto pixel_buffer_size = tab5.display_width() * 10; // tab5.display_height();
  if (!tab5.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // make the filter we'll use for the IMU to compute the orientation
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<2> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);
  static constexpr float beta = 0.5f; // higher = more accelerometer, lower = more gyro
  static espp::MadgwickFilter f(beta);

  using Imu = espp::M5StackTab5::Imu;
  auto kalman_filter_fn = [](float dt, const Imu::Value &accel,
                             const Imu::Value &gyro) -> Imu::Value {
    // Apply Kalman filter
    float accelRoll = atan2(accel.y, accel.z);
    float accelPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    kf.predict({espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y)}, dt);
    kf.update({accelRoll, accelPitch});
    float roll, pitch;
    std::tie(roll, pitch) = kf.get_state();
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = roll;
    orientation.pitch = pitch;
    orientation.yaw = 0.0f;
    return orientation;
  };

  auto madgwick_filter_fn = [](float dt, const Imu::Value &accel,
                               const Imu::Value &gyro) -> Imu::Value {
    // Apply Madgwick filter
    f.update(dt, accel.x, accel.y, accel.z, espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y),
             espp::deg_to_rad(gyro.z));
    float roll, pitch, yaw;
    f.get_euler(roll, pitch, yaw);
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = espp::deg_to_rad(roll);
    orientation.pitch = espp::deg_to_rad(pitch);
    orientation.yaw = espp::deg_to_rad(yaw);
    return orientation;
  };

  logger.info("Initializing IMU...");
  // initialize the IMU
  if (!tab5.initialize_imu(kalman_filter_fn)) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  // initialize the uSD card
  using SdCardConfig = espp::M5StackTab5::SdCardConfig;
  SdCardConfig sdcard_config{};
  if (!tab5.initialize_sdcard(sdcard_config)) {
    logger.warn("Failed to initialize uSD card, there may not be a uSD card inserted!");
  } else {
    uint32_t size_mb = 0;
    uint32_t free_mb = 0;
    if (tab5.get_sd_card_info(&size_mb, &free_mb)) {
      logger.info("uSD card size: {} MB, free space: {} MB", size_mb, free_mb);
    } else {
      logger.warn("Failed to get uSD card info");
    }
  }

  logger.info("Initializing RTC...");
  // initialize the RTC
  if (!tab5.initialize_rtc()) {
    logger.error("Failed to initialize RTC!");
    return;
  }

  auto current_time = std::tm{};
  if (!tab5.get_rtc_time(current_time)) {
    logger.error("Failed to get RTC time");
    return;
  }

  // only set the time if the year is before 2024
  if (current_time.tm_year < 124) {
    // set the RTC time to a known value (2024-01-15 14:30:45)
    // Set time using std::tm
    std::tm time = {};
    time.tm_year = 124; // 2024 - 1900
    time.tm_mon = 0;    // January (0-based)
    time.tm_mday = 15;  // 15th
    time.tm_hour = 14;  // 2 PM
    time.tm_min = 30;
    time.tm_sec = 45;
    time.tm_wday = 1; // Monday
    if (!tab5.set_rtc_time(time)) {
      logger.error("Failed to set RTC time");
      return;
    }
  } else {
    logger.info("RTC time is already set to a valid value {:%Y-%m-%d %H:%M:%S}", current_time);
  }

  logger.info("Initializing battery management...");
  // initialize battery monitoring
  if (!tab5.initialize_battery_monitoring()) {
    logger.error("Failed to initialize battery monitoring!");
    return;
  }

  // enable charging
  tab5.set_charging_enabled(true);

  logger.info("Initializing sound...");
  // initialize the sound
  if (!tab5.initialize_audio()) {
    logger.error("Failed to initialize sound!");
    return;
  }

  // unmute the audio and set the volume to 60% (do this before the GUI is
  // created so its volume label shows the right value)
  tab5.mute(false);
  tab5.volume(60.0f);

  // create the GUI: builds the UI (label, buttons, gravity lines, circle
  // layer) and starts the task which updates LVGL. All of its public methods
  // are thread-safe, so the touch callback, button callback, and data display
  // task below can call them directly.
  logger.info("Setting up LVGL UI...");
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  static const std::string instructions =
      fmt::format("Touch the screen to draw!\nPress the {} button to clear circles.\nPress the "
                  "{} button to rotate the display.\nPress the {} button to cycle the "
                  "brightness.\nThe Status and Audio tabs show the other subsystems.",
                  LV_SYMBOL_TRASH, LV_SYMBOL_REFRESH, LV_SYMBOL_EYE_OPEN);
  gui.set_label_text(instructions);

  // Brightness control with the hardware button: cycle through the same
  // 25/50/75/100% levels as the on-screen brightness button
  logger.info("Initializing button...");
  auto button_callback = [&](const auto &state) {
    logger.info("Button state: {}", state.active);
    if (state.active) {
      gui.cycle_brightness();
    }
  };
  if (!tab5.initialize_button(button_callback)) {
    logger.warn("Failed to initialize button");
  }

  // initialize the touchpad; each touch draws a circle (and plays a click
  // sound), while the touchscreen's button clears the circles
  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = tab5.touchpad_convert(touch);
    auto touchpad_data = tab5.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.debug("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        gui.clear_circles();
      }
      // if there is a touch point on the Draw tab, draw a circle and play a
      // click sound (touches on the other tabs go to their widgets)
      if (touchpad_data.num_touch_points > 0 && gui.draw_page_active()) {
        play_click(tab5);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  logger.info("Initializing touch...");
  // NOTE: this example raises the BSP interrupt-task stack size via
  // sdkconfig.defaults (CONFIG_M5STACK_TAB5_INTERRUPT_STACK_SIZE=8192); the
  // touch controller is read from that task and its error-logging path
  // needs more than the 4 KB BSP default. See the example README.
  if (!tab5.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touch!");
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
  tab5.audio_sample_rate(wav_sample_rate);

  // set the brightness to 75%
  tab5.brightness(75.0f);

  // Keep the analog microphone gain modest: the ES7210 front-end develops a
  // high-frequency whine as the analog gain is raised, so the loudness comes
  // from the RMS software makeup gain applied to the recording on stop (see
  // below) rather than from the analog stage. The mic +/- buttons still adjust
  // the analog gain if desired.
  tab5.microphone_volume(40.0f);

  // Allocate the recording buffer (16-bit interleaved stereo at the current
  // sample rate): prefer PSRAM, fall back to a couple of seconds in internal
  // RAM
  size_t bytes_per_second = tab5.audio_sample_rate() * 2 * sizeof(int16_t);
  recording_capacity = MAX_RECORDING_SECONDS * bytes_per_second;
  recording_buffer = static_cast<uint8_t *>(
      heap_caps_malloc(recording_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (recording_buffer == nullptr) {
    recording_capacity = FALLBACK_RECORDING_SECONDS * bytes_per_second;
    recording_buffer =
        static_cast<uint8_t *>(heap_caps_malloc(recording_capacity, MALLOC_CAP_8BIT));
  }
  if (recording_buffer == nullptr) {
    gui.set_audio_status("No recording buffer");
    logger.warn("Could not allocate a recording buffer; recording disabled");
    recording_capacity = 0;
  } else {
    logger.info("Recording buffer: {} KB ({} s at {} Hz stereo)", recording_capacity / 1024,
                recording_capacity / bytes_per_second, tab5.audio_sample_rate());
  }

  // The recording callback appends the recorded stereo frames to the buffer
  // and auto-stops when it is full (the main loop notices and updates the
  // GUI)
  auto record_data_callback = [](const uint8_t *data, size_t length) {
    if (!recording) {
      return;
    }
    size_t offset = recording_len;
    size_t to_copy = std::min(length, recording_capacity - offset);
    if (to_copy > 0) {
      memcpy(recording_buffer + offset, data, to_copy);
      recording_last_us = esp_timer_get_time();
      recording_len = offset + to_copy;
    }
    if (recording_len >= recording_capacity) {
      recording = false;
    }
  };

  // The record button toggles recording; the play button toggles playback of
  // the recording (streamed to the speaker by the main loop)
  gui.set_record_callback([&]() {
    if (recording_capacity == 0) {
      logger.warn("Recording unavailable (no buffer)");
      gui.set_audio_status("Recording unavailable (see log)");
      return;
    }
    if (recording) {
      recording = false; // the main loop stops the BSP recording and logs
    } else {
      playing = false;
      gui.set_play_active(false);
      recording_len = 0;
      recording_start_us = esp_timer_get_time();
      recording_last_us = recording_start_us.load();
      recording = true;
      tab5.start_audio_recording(record_data_callback);
      gui.set_record_active(true);
      gui.set_audio_status("Recording...");
    }
  });
  gui.set_play_callback([&]() {
    if (playing) {
      playing = false;
      gui.set_play_active(false);
      gui.set_audio_status("Playback stopped");
    } else if (recording_len > 0) {
      recording = false;
      playing = true;
      gui.set_play_active(true);
      gui.set_audio_status("Playing...");
    } else {
      logger.info("Nothing recorded yet; press the record button first");
      gui.set_audio_status("Nothing recorded yet");
    }
  });

  // make a task to read out various data such as IMU, battery monitoring, etc.
  // and print it to screen
  logger.info("Starting data display task...");
  espp::Task imu_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // sleep first in case we don't get IMU data and need to exit early
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 10ms);
         }
         static auto &tab5 = espp::M5StackTab5::get();
         static auto imu = tab5.imu();

         //////////////////////////////////////////////////////////////////////////
         // Update the Date/Time from the RTC
         //////////////////////////////////////////////////////////////////////////
         std::tm rtc_time;
         std::string rtc_text = "";
         if (tab5.get_rtc_time(rtc_time)) {
           rtc_text = fmt::format("\n{:%Y-%m-%d %H:%M:%S}\n", rtc_time);
         }

         //////////////////////////////////////////////////////////////////////////
         // Update the battery status
         //////////////////////////////////////////////////////////////////////////
         auto battery_status = tab5.read_battery_status();
         std::string battery_text =
             fmt::format("\nBattery: {:0.2f} V, {:0.1f} mA, {:0.1f} %, Charging: {}\n",
                         battery_status.voltage_v, battery_status.current_ma,
                         battery_status.charge_percent, battery_status.is_charging ? "Yes" : "No");

         auto now = esp_timer_get_time(); // time in microseconds
         static auto t0 = now;
         auto t1 = now;
         float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
         t0 = t1;

         //////////////////////////////////////////////////////////////////////////
         // Update the IMU data
         //////////////////////////////////////////////////////////////////////////
         std::error_code ec;
         // update the imu data
         if (!imu->update(dt, ec)) {
           return false;
         }
         // get accel
         auto accel = imu->get_accelerometer();
         auto gyro = imu->get_gyroscope();
         auto temp = imu->get_temperature();
         auto orientation = imu->get_orientation();
         auto gravity_vector = imu->get_gravity_vector();
         // invert the axes to convert from the sensor frame to the display's
         // natural (unrotated) frame
         gravity_vector.y = -gravity_vector.y;
         gravity_vector.x = -gravity_vector.x;

         // separator for imu
         std::string imu_text = "\nIMU Data:\n";
         imu_text += fmt::format("Accel: {:02.2f} {:02.2f} {:02.2f}\n", accel.x, accel.y, accel.z);
         imu_text += fmt::format("Gyro: {:03.2f} {:03.2f} {:03.2f}\n", espp::deg_to_rad(gyro.x),
                                 espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
         imu_text += fmt::format("Angle: {:03.2f} {:03.2f}\n", espp::rad_to_deg(orientation.roll),
                                 espp::rad_to_deg(orientation.pitch));
         imu_text += fmt::format("Temp: {:02.1f} C\n", temp);

         // Now show the madgwick filter's estimate of "down"
         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
         float roll = madgwick_orientation.roll;
         float pitch = madgwick_orientation.pitch;
         float vx = sin(pitch);
         float vy = -cos(pitch) * sin(roll);

         // invert the axes to convert from the sensor frame to the display's
         // natural (unrotated) frame
         vx = -vx;
         vy = -vy;

         std::string text;
         text += battery_text;
         text += rtc_text;
         text += imu_text;

         // update the GUI with the new data; the Gui handles remapping the
         // vectors for the current display rotation
         gui.set_status_text(text);
         gui.set_kalman_down(gravity_vector.x, gravity_vector.y);
         gui.set_madgwick_down(vx, vy);

         return false;
       },
       .task_config = {
           .name = "Data Display Task",
           .stack_size_bytes = 6 * 1024,
           .priority = 10,
           .core_id = 1,
       }});
  imu_task.start();

  // Main loop: stream any active playback to the speaker and notice when a
  // recording stops (either button press or the buffer filling up)
  size_t play_offset = 0;
  bool was_recording = false;
  while (true) {
    // feed the active playback in chunks, advancing by however much the
    // speaker's stream buffer accepted
    if (playing) {
      size_t len = recording_len;
      if (play_offset >= len) {
        playing = false;
        play_offset = 0;
        gui.set_play_active(false);
        gui.set_audio_status("Playback done");
        logger.info("Playback done");
      } else {
        play_offset += tab5.play_audio(recording_buffer + play_offset,
                                       std::min<size_t>(len - play_offset, 16384));
      }
    } else {
      play_offset = 0;
    }
    // notice when the recording stopped (button press or buffer full)
    bool now_recording = recording;
    if (was_recording && !now_recording) {
      tab5.stop_audio_recording();
      gui.set_record_active(false);
      gui.set_audio_status(fmt::format("Recorded {:.1f}s ({} plays)",
                                       static_cast<float>(recording_len) /
                                           (tab5.audio_sample_rate() * 2 * sizeof(int16_t)),
                                       LV_SYMBOL_PLAY));
      // report the measured capture rate: stereo frames recorded over the
      // wall clock they took to arrive should match the nominal sample rate
      size_t num_frames = recording_len / (2 * sizeof(int16_t));
      float elapsed_s = static_cast<float>(recording_last_us - recording_start_us) / 1e6f;
      float effective_hz = elapsed_s > 0.0f ? num_frames / elapsed_s : 0.0f;
      // Post-process the recording for playback on the Tab5's MONO speaker.
      // The ES7210 records 16-bit interleaved stereo (mic 1 -> left slot, mic 2
      // -> right); downmix each frame to the average (the speaker plays one I2S
      // slot, so the result is written to both). The captured level is low, so
      // rather than driving the analog gain hot (which whines), remove the DC
      // offset and apply an RMS-normalized software makeup gain: this makes the
      // recording play back at a consistent, audible level comparable to the
      // click WAV, without a high analog mic gain or a high speaker volume.
      auto *samples = reinterpret_cast<int16_t *>(recording_buffer);
      int16_t peak = 0;
      int64_t sum = 0;
      for (size_t i = 0; i < num_frames; i++) {
        int32_t mono = (static_cast<int32_t>(samples[2 * i]) + samples[2 * i + 1]) / 2;
        samples[2 * i] = static_cast<int16_t>(mono);
        peak = std::max<int16_t>(peak, static_cast<int16_t>(std::abs(mono)));
        sum += mono;
      }
      int32_t dc = num_frames ? static_cast<int32_t>(sum / static_cast<int64_t>(num_frames)) : 0;
      int64_t sum_sq = 0;
      for (size_t i = 0; i < num_frames; i++) {
        int32_t v = samples[2 * i] - dc;
        sum_sq += static_cast<int64_t>(v) * v;
      }
      double rms = num_frames ? std::sqrt(static_cast<double>(sum_sq) / num_frames) : 0.0;
      // target ~-15 dBFS leaves headroom; cap the gain so a near-silent capture
      // is not blown up into noise
      static constexpr double target_rms = 5500.0;
      double gain = rms > 1.0 ? std::clamp(target_rms / rms, 1.0, 64.0) : 1.0;
      for (size_t i = 0; i < num_frames; i++) {
        int32_t v = static_cast<int32_t>((samples[2 * i] - dc) * gain);
        int16_t mono = static_cast<int16_t>(std::clamp<int32_t>(v, -32768, 32767));
        samples[2 * i] = mono;
        samples[2 * i + 1] = mono;
      }
      logger.info("Recorded {} frames in {:.2f} s (~{:.0f} Hz effective, {} Hz nominal); "
                  "peak={} dc={} rms={:.0f}; applied makeup gain {:.1f}x",
                  num_frames, elapsed_s, effective_hz, tab5.audio_sample_rate(), peak, dc, rms,
                  gain);
    }
    was_recording = now_recording;
    std::this_thread::sleep_for(50ms);
  }
  //! [m5stack tab5 example]
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

static void play_click(espp::M5StackTab5 &tab5) {
  if (audio_bytes.size() > 0) {
    tab5.play_audio(audio_bytes);
  }
}
