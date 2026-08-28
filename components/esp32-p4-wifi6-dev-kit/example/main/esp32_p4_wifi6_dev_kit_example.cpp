/**
 * @file esp32_p4_wifi6_dev_kit_example.cpp
 * @brief Waveshare ESP32-P4-WIFI6-DEV-KIT BSP example
 *
 * Demonstrates the BSP: MIPI-DSI display + GT911 touch (draw circles and play a
 * click sound wherever you touch), microSD, audio (ES8311) record / playback,
 * the MIPI-CSI camera (live feed on the Camera tab), and Ethernet (IP101GRI)
 * as a DHCP client. Shows a live on-screen status read-out (panel, touch, SD,
 * Ethernet, and system memory/uptime).
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "format.hpp"

#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_timer.h>

#include "esp32-p4-wifi6-dev-kit.hpp"
#include "logger.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;
using Board = espp::Esp32P4Wifi6DevKit;

static std::vector<uint8_t> audio_bytes;
static bool load_audio(size_t &out_size, size_t &out_sample_rate);

// Audio recording state (written by the microphone callback, read/controlled
// from the GUI button callbacks and main loop). The recorded data is 16-bit
// mono at the audio sample rate.
static constexpr size_t MAX_RECORDING_SECONDS = 30;     // when PSRAM is available
static constexpr size_t FALLBACK_RECORDING_SECONDS = 2; // internal RAM fallback
static uint8_t *recording_buffer = nullptr;
static size_t recording_capacity = 0;
static std::atomic<bool> recording{false};
static std::atomic<size_t> recording_len{0};
static std::atomic<bool> playing{false};

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "ESP32-P4-WIFI6-DEV-KIT Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp32 p4 wifi6 dev kit example]
  auto &board = Board::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Display panel: {}", board.get_display_controller_name());

  // Probe the internal I2C bus (shared by the ES8311 codec, the GT911 touch
  // controller, and the camera SCCB).
  auto &i2c = board.internal_i2c();
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 128; addr++) {
    if (i2c.probe_device(addr)) {
      found.push_back(addr);
    }
  }
  logger.info("Found {} I2C device(s)", found.size());

  // Display (MIPI-DSI + configured panel driver)
  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  size_t pixel_buffer_size = board.display_width() * 50;
  if (!board.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // Build the GUI: a tabview with a Status tab (live subsystem state + rotate /
  // clear buttons; touch to draw circles), an Audio tab (record / play +
  // volume), and a Camera tab (live MIPI-CSI feed). All of its public methods
  // are thread-safe, so the touch, status and camera tasks below call them
  // directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});

  // On-screen status state. These are filled in as each subsystem initializes
  // below, and rendered immediately by the status task, so the display shows SD
  // / Ethernet coming online live instead of staying blank until the whole
  // bring-up finishes.
  static std::atomic<int> touch_x{0}, touch_y{0}, touch_n{0};
  static std::atomic<bool> sd_card_mounted{false};
  static std::atomic<uint32_t> sd_card_size_mb{0};
  static int64_t status_start_us = esp_timer_get_time();

  // Status updater: starts now (right after the display is up) and refreshes the
  // on-screen status ~10x/s. Ethernet state is read live from the board; SD
  // state is published into the atomics above as that subsystem comes up.
  espp::Task status_task(espp::Task::Config{
      .callback = [&board](std::mutex &m, std::condition_variable &cv) -> bool {
        const size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
        const size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
        const int uptime_s = static_cast<int>((esp_timer_get_time() - status_start_us) / 1'000'000);
        std::string eth_text = "(no link)";
        if (board.is_ethernet_connected()) {
          auto ip = board.ethernet_ip();
          eth_text = std::to_string(esp_ip4_addr1_16(&ip)) + "." +
                     std::to_string(esp_ip4_addr2_16(&ip)) + "." +
                     std::to_string(esp_ip4_addr3_16(&ip)) + "." +
                     std::to_string(esp_ip4_addr4_16(&ip));
        }
        std::string status =
            "Panel:    " + std::string(board.get_display_controller_name()) + " (" +
            std::to_string(board.display_width()) + "x" + std::to_string(board.display_height()) +
            ")\n" + "Touch:    " + std::to_string(touch_n.load()) + " pts (" +
            std::to_string(touch_x.load()) + ", " + std::to_string(touch_y.load()) + ")\n" +
            "SD card:  " +
            (sd_card_mounted ? std::to_string(sd_card_size_mb.load()) + " MB" : "none") + "\n" +
            "Ethernet: " + eth_text + "\n" + "Camera:   " + std::to_string(board.camera_width()) +
            "x" + std::to_string(board.camera_height()) + "\n" +
            "System:   " + std::to_string(free_internal) + " KB int, " +
            std::to_string(free_psram) + " KB psram free, up " + std::to_string(uptime_s) + " s";
        gui.set_status_text(status);
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 100ms);
        return false;
      },
      .task_config = {.name = "p4-wifi6 status", .stack_size_bytes = 6144}});
  status_task.start();

  // Touch: draw a circle wherever the screen is touched, and play a click on
  // each new touch-down. play_audio() is non-blocking, and the click is gated to
  // the touch-down edge so it doesn't retrigger every poll while held/dragging.
  static constexpr int kCircleRadius = 10;
  board.initialize_touch([&](const auto &data) {
    auto td = board.touchpad_convert(data);
    static Board::TouchpadData prev_td = {};
    touch_n = td.num_touch_points;
    touch_x = td.x;
    touch_y = td.y;
    if (td.num_touch_points > 0) {
      const bool new_touch = (prev_td != td);
      const bool touch_down_edge = (prev_td.num_touch_points == 0);
      // Touch feedback (click + circle) only applies on the draw/status page;
      // touches on the other tabs (buttons, sliders) stay silent.
      if (gui.draw_page_active()) {
        // Click feedback: instant on the touch-DOWN edge, and retriggered while
        // dragging - each retrigger restarts (clips) the click so drawing gives
        // a stream of overlapping-feel clicks. The retrigger interval keeps a
        // fast drag from restarting the click every poll (16 ms), which would
        // reduce it to a buzz of its first few milliseconds.
        static constexpr auto kClickRetriggerInterval = std::chrono::milliseconds(100);
        static auto last_click_time = std::chrono::steady_clock::time_point{};
        const auto now = std::chrono::steady_clock::now();
        const bool click_due =
            touch_down_edge || (now - last_click_time >= kClickRetriggerInterval);
        if (new_touch && click_due && !audio_bytes.empty()) {
          board.clear_audio();           // drop any queued tail (restart)
          board.play_audio(audio_bytes); // non-blocking
          last_click_time = now;
        }
        if (new_touch) {
          gui.draw_circle(td.x, td.y, kCircleRadius);
        }
      }
    }
    prev_td = td;
  });

  // microSD (optional — only present if a card is inserted)
  bool sd_ok = board.initialize_sdcard({.format_if_mount_failed = false});
  uint32_t sd_size_mb = 0, sd_free_mb = 0;
  if (sd_ok) {
    board.get_sd_card_info(&sd_size_mb, &sd_free_mb);
    logger.info("SD card: {} MB total, {} MB free", sd_size_mb, sd_free_mb);
  } else {
    logger.warn("No SD card mounted");
  }
  sd_card_mounted = sd_ok;
  sd_card_size_mb = sd_size_mb; // published to the status task

  // Audio (ES8311) — load the embedded click sound first so we can initialize
  // the codec directly at the clip's sample rate (changing the sample rate after
  // the audio task is running is racy, so we avoid it here).
  size_t wav_size = 0, wav_sample_rate = 0;
  bool have_audio = load_audio(wav_size, wav_sample_rate);
  uint32_t audio_rate = have_audio ? static_cast<uint32_t>(wav_sample_rate) : 48000;
  if (board.initialize_audio(audio_rate)) {
    board.mute(false);
    board.volume(60.0f);
    if (have_audio) {
      logger.info("Loaded {} bytes of click audio @ {} Hz", wav_size, wav_sample_rate);
    }

    // Microphone: the ES8311 is full duplex, so the onboard microphone records
    // at the speaker's sample rate. Buffer the recorded mono samples and
    // auto-stop when the buffer is full (the main loop notices and updates the
    // GUI).
    auto mic_callback = [](const uint8_t *data, size_t num_bytes) {
      if (!recording) {
        return;
      }
      size_t offset = recording_len;
      size_t to_copy = std::min(num_bytes, recording_capacity - offset);
      if (to_copy > 0) {
        memcpy(recording_buffer + offset, data, to_copy);
        recording_len = offset + to_copy;
      }
      if (recording_len >= recording_capacity) {
        recording = false;
      }
    };
    if (board.initialize_microphone(mic_callback)) {
      // allocate the recording buffer (16-bit mono at the current sample rate):
      // prefer PSRAM, fall back to a couple of seconds in internal RAM
      size_t bytes_per_second = board.audio_sample_rate() * sizeof(int16_t);
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
        logger.info("Recording buffer: {} KB ({} s at {} Hz mono)", recording_capacity / 1024,
                    recording_capacity / bytes_per_second, board.audio_sample_rate());
      }
    } else {
      logger.warn("Could not initialize the microphone!");
      gui.set_audio_status("Mic unavailable (see log)");
    }
  }

  // The record button toggles recording; the play button toggles playback of
  // the recording (streamed to the speaker by the main loop).
  gui.set_record_callback([&]() {
    if (recording_capacity == 0) {
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

  // Ethernet (IP101GRI) — DHCP client; the callback fires once an IP is acquired
  board.initialize_ethernet({
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            char buf[16] = {0};
            esp_ip4addr_ntoa(&ip, buf, sizeof(buf));
            logger.info("Ethernet IP: {}", buf);
          },
  });

  // Camera (MIPI-CSI) — stream each RGB565 frame to the Camera tab. The BSP runs
  // a capture task that hands each frame to this callback; forward it to the
  // thread-safe GUI. Non-fatal: the rest of the example still runs if the camera
  // is unavailable.
  logger.info("Initializing camera...");
  if (!board.initialize_camera(
          [&](const uint8_t *data, int w, int h, size_t) { gui.set_camera_frame(data, w, h); })) {
    logger.warn("Failed to initialize camera; the Camera tab will stay blank");
  }
  //! [esp32 p4 wifi6 dev kit example]

  // Main loop: stream any active playback to the speaker in chunks and notice
  // when a recording stops (either button press or the buffer filling up).
  size_t play_offset = 0;
  bool was_recording = false;
  while (true) {
    if (playing) {
      size_t len = recording_len;
      if (play_offset >= len) {
        playing = false;
        play_offset = 0;
        gui.set_play_active(false);
        gui.set_audio_status("Playback done");
        logger.info("Playback done");
      } else {
        play_offset += board.play_audio(recording_buffer + play_offset,
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
                                           (board.audio_sample_rate() * sizeof(int16_t)),
                                       LV_SYMBOL_PLAY));
      logger.info("Recorded {} bytes", recording_len.load());
    }
    was_recording = now_recording;

    std::this_thread::sleep_for(20ms);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Load the embedded click.wav (stripping the 44-byte WAV header) and report its
// size and sample rate.
//////////////////////////////////////////////////////////////////////////////
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
  uint32_t sample_rate = 0;
  std::memcpy(&sample_rate, &audio_bytes[24], sizeof(sample_rate));
  // Walk the RIFF chunks to find the 'data' chunk and keep exactly its payload.
  // A fixed 44-byte strip is wrong for files with trailing metadata chunks
  // (cue/LIST/bext): those bytes would be played as audio, producing a pop at
  // the end of playback.
  size_t data_off = 0, data_len = 0;
  for (size_t off = 12; off + 8 <= audio_bytes.size();) {
    uint32_t chunk_size = 0;
    std::memcpy(&chunk_size, &audio_bytes[off + 4], sizeof(chunk_size));
    if (std::memcmp(&audio_bytes[off], "data", 4) == 0) {
      data_off = off + 8;
      data_len = std::min<size_t>(chunk_size, audio_bytes.size() - data_off);
      break;
    }
    off += 8 + chunk_size + (chunk_size & 1); // chunks are word-aligned
  }
  if (data_len == 0) {
    audio_bytes.clear();
    return false;
  }
  audio_bytes.erase(audio_bytes.begin() + data_off + data_len, audio_bytes.end());
  audio_bytes.erase(audio_bytes.begin(), audio_bytes.begin() + data_off);
  out_size = audio_bytes.size();
  out_sample_rate = sample_rate;
  return true;
}
