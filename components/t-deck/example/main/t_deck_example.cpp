#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <stdlib.h>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "t-deck.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

static std::vector<uint8_t> audio_bytes;

static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::TDeck &tdeck);
static void resample_click(uint32_t from_rate, uint32_t to_rate);

// Run all audio at 16 kHz - the rate LilyGO's own T-Deck firmware uses for
// the ES7210 (the codec sounds clean there, and higher rates on this board
// produce a distorted / robotic capture). The speaker plays at the same rate
// so a recording plays back at the correct pitch with no runtime rate
// switching; the bundled 44.1 kHz click is resampled to 16 kHz at load.
static constexpr uint32_t AUDIO_SAMPLE_RATE_HZ = 16000;
static constexpr uint32_t MIC_SAMPLE_RATE_HZ = AUDIO_SAMPLE_RATE_HZ;

// Audio recording state (written by the microphone callback, read/controlled
// from the GUI / keyboard callbacks and main loop). The recorded data is
// 16-bit interleaved stereo at the microphone sample rate.
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
// number of bytes to discard at the start of a recording: the I2S RX DMA and
// the ES7210 emit a burst of settling garbage right after a recording begins,
// which otherwise plays back as a click of static at the front of the clip
static std::atomic<size_t> warmup_bytes_remaining{0};
// record / play toggles, shared by the on-screen buttons and the keyboard
// (assigned once the microphone is initialized)
static std::function<void()> toggle_record;
static std::function<void()> toggle_play;

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
  // initialize the sound at the example's fixed rate (see AUDIO_SAMPLE_RATE_HZ)
  // so the speaker runs at that rate from the start - no runtime rate
  // reconfiguration, which is where a speaker-vs-recording pitch mismatch
  // could creep in
  if (!tdeck.initialize_sound(AUDIO_SAMPLE_RATE_HZ)) {
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
  gui.set_label_text(
      fmt::format("Touch the screen to draw!\nPress the delete key or the {} button to clear "
                  "circles.\nPress the space key or the {} button to rotate the display.\n"
                  "The Audio tab (or the 'r' / 'p' keys) records and plays back audio; "
                  "'n' / '$' / 'm' adjust / mute the speaker volume.",
                  LV_SYMBOL_TRASH, LV_SYMBOL_REFRESH));

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
      gui.refresh_audio_label();
    } else if (key == '$') {
      // '$' key will increase audio volume (right of 'm' key)
      logger.info("Increasing volume");
      tdeck.volume(tdeck.volume() + 10.0f);
      logger.info("Volume: {}", tdeck.volume());
      gui.refresh_audio_label();
    } else if (key == 'r') {
      // 'r' key toggles recording from the microphones
      if (toggle_record) {
        toggle_record();
      }
    } else if (key == 'p') {
      // 'p' key toggles playback of the recording
      if (toggle_play) {
        toggle_play();
      }
    }
  };
  bool start_task = true;
  if (!tdeck.initialize_keyboard(start_task, keypress_callback)) {
    logger.error("Failed to initialize Keyboard!");
    return;
  }

  // initialize the trackball. This example uses the microphone, and per
  // LilyGO's T-Deck documentation GPIO0 (the trackball's center / click
  // button, shared with BOOT) is not available while the microphone is
  // enabled - leaving it configured produces spurious interrupts that jitter
  // the audio capture (a robotic / staticy recording). So initialize the
  // trackball with the center button disabled; the four directional pins
  // still work.
  auto trackball_callback = [&](const auto &trackball) {
    logger.debug("Trackball: {}", trackball);
  };
  if (!tdeck.initialize_trackball(trackball_callback, 10, /*enable_center_button=*/false)) {
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
      // if there is a touch point on the Draw tab, draw a circle and play a
      // click sound (touches on the other tabs go to their widgets)
      if (touchpad_data.num_touch_points > 0 && gui.draw_page_active()) {
        play_click(tdeck);
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  // NOTE: this example raises the BSP interrupt-task stack size via
  // sdkconfig.defaults (CONFIG_TDECK_INTERRUPT_STACK_SIZE=8192); the touch
  // controller is read from that task and its error-logging path needs
  // more than the 4 KB BSP default. See the example README.
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
  logger.info("Loaded {} bytes of audio ({} Hz)", wav_size, wav_sample_rate);

  // Run the whole example at 16 kHz (see AUDIO_SAMPLE_RATE_HZ). The speaker
  // was already initialized at that rate; the bundled click was decoded at
  // its native rate, so resample it to 16 kHz once here so it plays at the
  // right pitch through the 16 kHz speaker.
  resample_click(static_cast<uint32_t>(wav_sample_rate), AUDIO_SAMPLE_RATE_HZ);

  // unmute the audio and set the volume to 20%
  tdeck.mute(false);
  tdeck.volume(20.0f);

  // set the display brightness to be 75%
  tdeck.brightness(75.0f);

  // Initialize the microphones (the ES7210 is on its own I2S bus, so pick
  // the speaker's sample rate to make the recording directly playable) and
  // buffer the recorded stereo frames; the recording auto-stops when the
  // buffer is full and the main loop notices and updates the GUI
  auto mic_callback = [](const uint8_t *data, size_t num_bytes) {
    if (!recording) {
      return;
    }
    // drop the warm-up (settling) bytes at the very start of the recording
    size_t warm = warmup_bytes_remaining;
    if (warm > 0) {
      if (num_bytes <= warm) {
        warmup_bytes_remaining = warm - num_bytes;
        return;
      }
      data += warm;
      num_bytes -= warm;
      warmup_bytes_remaining = 0;
    }
    int64_t now = esp_timer_get_time();
    // stamp the true start of retained audio on the first kept sample so the
    // measured capture rate excludes the warm-up period
    if (recording_start_us == 0) {
      recording_start_us = now;
    }
    size_t offset = recording_len;
    size_t to_copy = std::min(num_bytes, recording_capacity - offset);
    if (to_copy > 0) {
      memcpy(recording_buffer + offset, data, to_copy);
      recording_last_us = now;
      recording_len = offset + to_copy;
    }
    if (recording_len >= recording_capacity) {
      recording = false;
    }
  };
  // record at the ES7210's comfortable 16 kHz (see MIC_SAMPLE_RATE_HZ), which
  // is independent of the speaker's rate (the ES7210 is on its own I2S bus)
  bool have_mic = tdeck.initialize_microphone(mic_callback, MIC_SAMPLE_RATE_HZ);
  if (have_mic) {
    // The T-Deck's electret mics are genuinely low-sensitivity (LilyGO's own
    // firmware reads only ~200 counts for loud speech next to the mic), so the
    // analog stage is driven fairly hard here (~30 dB). Earlier this railed the
    // ADC, but that was the mic's DC offset being amplified with the ES7210
    // high-pass filter disabled; the driver now enables the HPF, so the signal
    // swings symmetrically around zero and this gain no longer saturates. The
    // software auto-gain applied on stop (see the RMS normalization below)
    // makes up whatever level remains; the mic +/- buttons adjust from here.
    tdeck.microphone_volume(70.0f);
    // allocate the recording buffer (16-bit interleaved stereo): prefer
    // PSRAM, fall back to a couple of seconds in internal RAM
    size_t bytes_per_second = tdeck.microphone_sample_rate() * 2 * sizeof(int16_t);
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
                  recording_capacity / bytes_per_second, tdeck.microphone_sample_rate());
    }
  } else {
    logger.warn("Could not initialize the microphone!");
    gui.set_audio_status("Mic unavailable (see log)");
  }

  // The record button / 'r' key toggles recording; the play button / 'p' key
  // toggles playback of the recording (streamed to the speaker by the main
  // loop)
  toggle_record = [&]() {
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
      // discard the first ~250 ms of settling garbage; recording_start_us is
      // stamped by the mic callback on the first retained sample
      warmup_bytes_remaining = (tdeck.microphone_sample_rate() / 4) * 2 * sizeof(int16_t);
      recording_start_us = 0;
      recording_last_us = 0;
      recording = true;
      gui.set_record_active(true);
      gui.set_audio_status("Recording...");
    }
  };
  toggle_play = [&]() {
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
      logger.info("Nothing recorded yet; press the record button (or 'r') first");
      gui.set_audio_status("Nothing recorded yet");
    }
  };
  gui.set_record_callback(toggle_record);
  gui.set_play_callback(toggle_play);

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
        // Top up the speaker's stream buffer until it is full (play_audio
        // returns less than requested once it can't accept more). A single
        // chunk per loop leaves the buffer able to drain between iterations,
        // which underruns and sounds like static / dropouts during a
        // continuous playback.
        while (play_offset < len) {
          size_t chunk = std::min<size_t>(len - play_offset, 16384);
          size_t queued = tdeck.play_audio(recording_buffer + play_offset, chunk);
          play_offset += queued;
          if (queued < chunk) {
            break; // stream buffer full for now
          }
        }
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
                                           (tdeck.microphone_sample_rate() * 2 * sizeof(int16_t)),
                                       LV_SYMBOL_PLAY));
      // report the measured capture rate: stereo frames recorded over the
      // wall clock they took to arrive should match the nominal sample rate
      size_t num_frames = recording_len / (2 * sizeof(int16_t));
      float elapsed_s = static_cast<float>(recording_last_us - recording_start_us) / 1e6f;
      float effective_hz = elapsed_s > 0.0f ? num_frames / elapsed_s : 0.0f;
      // Post-process the recording for playback on the T-Deck's MONO speaker.
      // Use a single microphone (MIC1, the left slot - summing the two
      // spatially separated mics comb-filters the sound) and mirror it to both
      // stereo channels. The processing chain is: (1) de-glitch the electrical
      // impulse noise, (2) remove the DC offset, (3) apply an RMS-normalized
      // software makeup gain. The T-Deck mic is low-sensitivity (LilyGO's own
      // firmware sees only ~200 counts for loud speech), so the makeup gain -
      // not the analog stage - provides the loudness; RMS (rather than peak)
      // sets it so a residual glitch cannot collapse the gain, and clipping
      // catches any amplified outliers.
      auto *samples = reinterpret_cast<int16_t *>(recording_buffer);
      // De-glitch: the ES7210 capture on this board carries random, bursty
      // electrical impulse noise - roughly 0.3% of samples jump to |v| ~ 8000+
      // in runs of one to a few samples. (The de-spike frame gaps were measured
      // to be non-periodic, i.e. line noise coupling into the mic / I2S rather
      // than a framing or DMA-boundary artifact, so it cannot be removed by any
      // codec register and has to be concealed here.) Detect each glitch by its
      // large deviation from a local 7-point median (which tracks the real,
      // slew-limited waveform, so legitimate speech is never flagged) and then
      // linearly interpolate across each contiguous bad run from the good
      // samples on either side. Interpolating a whole run conceals short bursts
      // that a single-sample median replacement would leave behind. Done before
      // the RMS/gain measurement so the glitches cannot inflate the makeup gain.
      static constexpr int32_t kSpikeDelta = 2500;
      size_t despiked = 0;
      std::vector<uint8_t> bad(num_frames, 0);
      for (size_t i = 3; i + 3 < num_frames; i++) {
        int32_t w[7];
        for (int k = 0; k < 7; k++) {
          w[k] = samples[2 * (i - 3 + k)];
        }
        for (int a = 1; a < 7; a++) { // insertion sort the 7-sample window
          int32_t key = w[a];
          int b = a - 1;
          while (b >= 0 && w[b] > key) {
            w[b + 1] = w[b];
            b--;
          }
          w[b + 1] = key;
        }
        int32_t median = w[3];
        if (std::abs(static_cast<int32_t>(samples[2 * i]) - median) > kSpikeDelta) {
          bad[i] = 1;
        }
      }
      for (size_t i = 0; i < num_frames;) {
        if (!bad[i]) {
          i++;
          continue;
        }
        size_t j = i; // [i, j) is a contiguous run of bad samples
        while (j < num_frames && bad[j]) {
          j++;
        }
        int32_t left = (i > 0) ? samples[2 * (i - 1)] : 0;
        int32_t right = (j < num_frames) ? samples[2 * j] : left;
        int32_t span = static_cast<int32_t>(j - i) + 1;
        for (size_t k = i; k < j; k++) {
          int32_t t = static_cast<int32_t>(k - i) + 1;
          samples[2 * k] = static_cast<int16_t>(left + (right - left) * t / span);
        }
        despiked += j - i;
        i = j;
      }
      int16_t min_l = 32767, max_l = -32768;
      int64_t sum_l = 0, sum_sq = 0;
      for (size_t i = 0; i < num_frames; i++) {
        int16_t l = samples[2 * i]; // MIC1 (left)
        min_l = std::min(min_l, l);
        max_l = std::max(max_l, l);
        sum_l += l;
      }
      int32_t dc_l =
          num_frames ? static_cast<int32_t>(sum_l / static_cast<int64_t>(num_frames)) : 0;
      for (size_t i = 0; i < num_frames; i++) {
        int32_t v = samples[2 * i] - dc_l;
        sum_sq += static_cast<int64_t>(v) * v;
      }
      double rms = num_frames ? std::sqrt(static_cast<double>(sum_sq) / num_frames) : 0.0;
      // target RMS ~4000 (about -18 dBFS) leaves headroom; cap the gain so a
      // near-silent recording is not blown up into noise
      static constexpr double target_rms = 4000.0;
      double gain = rms > 1.0 ? std::clamp(target_rms / rms, 1.0, 64.0) : 1.0;
      for (size_t i = 0; i < num_frames; i++) {
        int32_t v = static_cast<int32_t>((samples[2 * i] - dc_l) * gain);
        int16_t mono = static_cast<int16_t>(std::clamp<int32_t>(v, -32768, 32767));
        samples[2 * i] = mono;
        samples[2 * i + 1] = mono; // mono: duplicate MIC1 to both channels
      }
      logger.info("Recorded {} frames in {:.2f} s (~{:.0f} Hz effective, {} Hz nominal)",
                  num_frames, elapsed_s, effective_hz, tdeck.microphone_sample_rate());
      logger.info("  MIC1: raw min={} max={} dc={} rms={:.0f}; de-spiked {} samples; "
                  "applied software gain {:.1f}x",
                  min_l, max_l, dc_l, rms, despiked, gain);
    }
    was_recording = now_recording;
    // tick faster while playing so the stream buffer is topped up well before
    // it can drain (avoids underrun static); idle more slowly otherwise
    std::this_thread::sleep_for(playing ? 10ms : 50ms);
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
  // Enqueue the click without blocking the caller (this runs in the touch
  // callback). play_audio() enqueues only whole frames and returns how many
  // bytes it took, so advance by that count - advancing by the requested size
  // would skip samples (at 16 kHz audio_buffer_size is not a multiple of a
  // 4-byte frame, so this matters here). Stop as soon as the stream buffer is
  // full rather than waiting for it to drain; blocking here would freeze the
  // touch task for the whole click. The click comfortably fits in the stream
  // buffer, so it plays in full in practice.
  auto audio_buffer_size = tdeck.audio_buffer_size();
  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    size_t chunk = std::min(audio_buffer_size, audio_bytes.size() - offset);
    size_t queued = tdeck.play_audio(audio_bytes.data() + offset, chunk);
    offset += queued;
    if (queued < chunk) {
      break; // stream buffer full for now; do not block the caller
    }
  }
}

// Resample the loaded click (16-bit interleaved stereo) in place from
// from_rate to to_rate with linear interpolation, so it plays at the correct
// pitch through the speaker running at to_rate.
static void resample_click(uint32_t from_rate, uint32_t to_rate) {
  if (from_rate == to_rate || audio_bytes.empty() || to_rate == 0) {
    return;
  }
  size_t in_frames = audio_bytes.size() / (2 * sizeof(int16_t));
  if (in_frames < 2) {
    return;
  }
  // Copy the samples into a properly-aligned int16_t buffer: audio_bytes is a
  // byte vector, so reinterpreting its storage as int16_t* and dereferencing it
  // would be unaligned / undefined behavior.
  std::vector<int16_t> in(in_frames * 2);
  std::memcpy(in.data(), audio_bytes.data(), in.size() * sizeof(int16_t));
  size_t out_frames = static_cast<size_t>(static_cast<uint64_t>(in_frames) * to_rate / from_rate);
  std::vector<int16_t> out(out_frames * 2);
  double step = static_cast<double>(from_rate) / to_rate;
  for (size_t j = 0; j < out_frames; ++j) {
    double src = j * step;
    size_t i0 = static_cast<size_t>(src);
    size_t i1 = std::min(i0 + 1, in_frames - 1);
    double frac = src - i0;
    for (int ch = 0; ch < 2; ++ch) {
      double a = in[2 * i0 + ch];
      double b = in[2 * i1 + ch];
      out[2 * j + ch] = static_cast<int16_t>(a + (b - a) * frac);
    }
  }
  audio_bytes.resize(out.size() * sizeof(int16_t));
  std::memcpy(audio_bytes.data(), out.data(), audio_bytes.size());
}
