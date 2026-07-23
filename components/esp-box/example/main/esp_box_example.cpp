#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>
#include <utility>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "esp-box.hpp"

#include "gui.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

static std::vector<uint8_t> audio_bytes;

static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::EspBox &box);

// Audio recording state (written by the microphone callback, read/controlled
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
  espp::Logger logger({.tag = "ESP BOX Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp box example]
  espp::EspBox &box = espp::EspBox::get();
  box.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Running on {}", box.box_type());

  // initialize the sound
  if (!box.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  // initialize the LCD
  if (!box.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = box.lcd_width() * 50;
  // initialize the LVGL display for the esp-box
  if (!box.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // make the filter we'll use for the IMU to compute the orientation
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<2> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);
  static constexpr float beta = 0.1f; // higher = more accelerometer, lower = more gyro
  static espp::MadgwickFilter f(beta);

  using Imu = espp::EspBox::Imu;
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

  // initialize the IMU
  if (!box.initialize_imu(kalman_filter_fn)) {
    logger.error("Failed to initialize IMU!");
    return;
  }

  // create the GUI: builds the UI (labels, buttons, gravity lines, circle
  // layer) and starts the task which updates LVGL. All of its public methods
  // are thread-safe, so the touch callback and IMU task below can call them
  // directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  static const std::string instructions =
      fmt::format("Touch the screen to draw!\nPress the home button or the {} button to clear "
                  "circles.\nPress the {} button to rotate the display.\nThe IMU and Audio "
                  "tabs show the other subsystems.",
                  LV_SYMBOL_TRASH, LV_SYMBOL_REFRESH);
  gui.set_label_text(instructions);

  // initialize the touchpad; each touch draws a circle (and plays a click
  // sound), while the home button clears the circles
  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = box.touchpad_convert(touch);
    auto touchpad_data = box.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the home button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        gui.clear_circles();
      }
      // if there is a touch point on the Draw tab, draw a circle and play a
      // click sound (touches on the other tabs go to their widgets)
      if (touchpad_data.num_touch_points > 0 && gui.draw_page_active()) {
        play_click(box);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  // NOTE: this example raises the BSP interrupt-task stack size via
  // sdkconfig.defaults (CONFIG_ESP_BOX_INTERRUPT_STACK_SIZE=8192); the
  // touch controller is read from that task and its error-logging path
  // needs more than the 4 KB BSP default. See the example README.
  if (!box.initialize_touch(touch_callback)) {
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
  box.audio_sample_rate(wav_sample_rate);

  // unmute the audio and set the volume to 60%
  box.mute(false);
  box.volume(60.0f);

  // set the display brightness to be 75%
  box.brightness(75.0f);

  // Initialize the microphones (the ES7210 shares the I2S bus with the
  // speaker in full duplex, so recording runs at the speaker's sample rate)
  // and buffer the recorded stereo frames; the recording auto-stops when the
  // buffer is full and the main loop notices and updates the GUI
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
      recording = false;
    }
  };
  bool have_mic = box.initialize_microphone(mic_callback);
  if (!have_mic) {
    gui.set_audio_status("Mic unavailable (see log)");
  }
  if (have_mic) {
    // start at a modest microphone gain: the BSP default is fairly hot and
    // can clip on close / loud sound. Nudge it up with the mic + button if
    // recordings are too quiet.
    box.microphone_volume(40.0f);
    // allocate the recording buffer (16-bit interleaved stereo at the
    // current sample rate): prefer PSRAM, fall back to a couple of seconds
    // in internal RAM
    size_t bytes_per_second = box.audio_sample_rate() * 2 * sizeof(int16_t);
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
      gui.set_audio_status("No recording buffer");
      recording_capacity = 0;
    } else {
      logger.info("Recording buffer: {} KB ({} s at {} Hz stereo)", recording_capacity / 1024,
                  recording_capacity / bytes_per_second, box.audio_sample_rate());
    }
  } else {
    logger.warn("Could not initialize the microphone!");
  }

  // The record button toggles recording; the play button toggles playback of
  // the recording (streamed to the speaker by the main loop)
  gui.set_record_callback([&]() {
    if (!have_mic || recording_capacity == 0) {
      logger.warn("Recording unavailable (no microphone / no buffer)");
      gui.set_audio_status("Mic unavailable (see log)");
      return;
    }
    if (recording) {
      recording = false; // the main loop notices and logs the summary
    } else {
      playing = false;
      gui.set_play_active(false);
      recording_len = 0;
      recording_start_us = esp_timer_get_time();
      recording_last_us = recording_start_us;
      recording = true;
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

  // make a task to read out the IMU data and update the GUI with it
  espp::Task imu_task(
      {.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
         // sleep first in case we don't get IMU data and need to exit early
         {
           std::unique_lock<std::mutex> lock(m);
           cv.wait_for(lock, 10ms);
         }
         static auto &box = espp::EspBox::get();
         static auto imu = box.imu();

         auto now = esp_timer_get_time(); // time in microseconds
         static auto t0 = now;
         auto t1 = now;
         float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
         t0 = t1;

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

         auto box_type = box.box_type();
         if (box_type == espp::EspBox::BoxType::BOX) {
           std::swap(gravity_vector.x, gravity_vector.y);
           gravity_vector.y = -gravity_vector.y;
         }

         std::string text;
         text += fmt::format("Accel: {:02.2f} {:02.2f} {:02.2f}\n", accel.x, accel.y, accel.z);
         text += fmt::format("Gyro: {:03.2f} {:03.2f} {:03.2f}\n", espp::deg_to_rad(gyro.x),
                             espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
         text += fmt::format("Angle: {:03.2f} {:03.2f}\n", espp::rad_to_deg(orientation.roll),
                             espp::rad_to_deg(orientation.pitch));
         text += fmt::format("Temp: {:02.1f} C\n", temp);

         // Now show the madgwick filter's estimate of "down"
         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
         float roll = madgwick_orientation.roll;
         float pitch = madgwick_orientation.pitch;
         float vx = sin(pitch);
         float vy = -cos(pitch) * sin(roll);

         if (box_type == espp::EspBox::BoxType::BOX) {
           std::swap(vx, vy);
           vy = -vy;
         }

         // update the GUI with the new data; the Gui handles remapping the
         // vectors for the current display rotation
         gui.set_imu_text(text);
         gui.set_kalman_down(gravity_vector.x, gravity_vector.y);
         gui.set_madgwick_down(vx, vy);

         return false;
       },
       .task_config = {
           .name = "IMU",
           .stack_size_bytes = 6 * 1024,
           .priority = 10,
           .core_id = 0,
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
        play_offset += box.play_audio(recording_buffer + play_offset,
                                      std::min<size_t>(len - play_offset, 16384));
      }
    } else {
      play_offset = 0;
    }
    // notice when the recording stopped (button press or buffer full)
    bool now_recording = recording;
    if (was_recording && !now_recording) {
      gui.set_record_active(false);
      gui.set_audio_status(fmt::format("Recorded {:.1f}s ({} plays)",
                                       static_cast<float>(recording_len) /
                                           (box.audio_sample_rate() * 2 * sizeof(int16_t)),
                                       LV_SYMBOL_PLAY));
      // report the measured capture rate: stereo frames recorded over the
      // wall clock they took to arrive should match the nominal sample rate
      size_t num_frames = recording_len / (2 * sizeof(int16_t));
      float elapsed_s = static_cast<float>(recording_last_us - recording_start_us) / 1e6f;
      float effective_hz = elapsed_s > 0.0f ? num_frames / elapsed_s : 0.0f;
      // Post-process the recording for playback on the box's MONO speaker: the
      // ES7210 records 16-bit interleaved stereo (mic 1 -> left slot, mic 2 ->
      // right), but the speaker only plays one I2S slot, so audio captured on
      // the other slot would be inaudible. Downmix each L/R frame to the
      // average and write it to BOTH slots so the mono speaker always plays
      // it. Also report the peak amplitude per channel: a peak near 0 means
      // the microphones captured silence (a codec / wiring problem), while a
      // healthy peak means capture is fine.
      auto *samples = reinterpret_cast<int16_t *>(recording_buffer);
      int16_t peak_left = 0, peak_right = 0;
      for (size_t i = 0; i < num_frames; i++) {
        int16_t l = samples[2 * i];
        int16_t r = samples[2 * i + 1];
        peak_left = std::max<int16_t>(peak_left, static_cast<int16_t>(std::abs(l)));
        peak_right = std::max<int16_t>(peak_right, static_cast<int16_t>(std::abs(r)));
        int16_t mono = static_cast<int16_t>((static_cast<int32_t>(l) + r) / 2);
        samples[2 * i] = mono;
        samples[2 * i + 1] = mono;
      }
      logger.info("Recorded {} frames in {:.2f} s (~{:.0f} Hz effective, {} Hz nominal); "
                  "peak L={} R={} (of 32767 - near 0 means the mics captured silence)",
                  num_frames, elapsed_s, effective_hz, box.audio_sample_rate(), peak_left,
                  peak_right);
    }
    was_recording = now_recording;
    std::this_thread::sleep_for(50ms);
  }
  //! [esp box example]
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

static void play_click(espp::EspBox &box) {
  // use the box.play_audio() function to play a sound, breaking it into
  // audio_buffer_size chunks
  auto audio_buffer_size = box.audio_buffer_size();
  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    size_t bytes_to_play = std::min(audio_buffer_size, audio_bytes.size() - offset);
    box.play_audio(audio_bytes.data() + offset, bytes_to_play);
    offset += bytes_to_play;
  }
}
