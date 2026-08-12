/**
 * @file esp32_p4_function_ev_board_example.cpp
 * @brief ESP32-P4 Function EV Board BSP example
 *
 * Demonstrates the BSP: MIPI-DSI display + GT911 touch (draw circles and play a
 * click sound wherever you touch), microSD, audio (ES8311), Ethernet (IP101)
 * with an RTPS publisher, and the BOOT button. Shows a live on-screen status
 * read-out (panel, touch, SD, Ethernet, RTPS, and system memory/uptime).
 */

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_netif.h>
#include <esp_timer.h>

#include "cdr.hpp"
#include "ping.hpp"
#include "rtps.hpp"

#include "esp32-p4-function-ev-board.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;
using Board = espp::Esp32P4FunctionEvBoard;

// Address of the most recently discovered RTPS peer (filled by the participant's
// on_participant_discovered callback, read by the ping self-test).
static std::mutex g_peer_mutex;
static std::string g_peer_addr;

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
// wall-clock bounds of the capture, for reporting the measured effective
// sample rate (ordering is provided by the `recording` atomic)
static std::atomic<int64_t> recording_start_us{0};
static std::atomic<int64_t> recording_last_us{0};

// Ping a target host a few times and log the result (uses the espp Ping helper).
static void ping_target(espp::Logger &logger, const char *name, const std::string &ip) {
  if (ip.empty() || ip == "0.0.0.0") {
    logger.warn("Ping {}: no address to ping", name);
    return;
  }
  logger.info("Pinging {} ({})...", name, ip);
  std::error_code ec;
  espp::Ping ping(espp::Ping::Config{
      .session = {.target_host = ip, .count = 4, .interval_ms = 500, .timeout_ms = 1000},
      .callbacks = {
          .on_session_start = nullptr,
          .on_reply =
              [&logger, name](uint32_t seq, uint32_t ttl, uint32_t time_ms, uint32_t bytes) {
                logger.info("  {}: seq={} ttl={} time={}ms ({} bytes)", name, seq, ttl, time_ms,
                            bytes);
              },
          .on_timeout = [&logger, name]() { logger.warn("  {}: request timed out", name); },
          .on_end =
              [&logger, name, ip](const espp::Ping::Stats &s) {
                logger.info("Ping {} ({}): {}/{} received, {:.0f}% loss, avg {} ms", name, ip,
                            s.received, s.transmitted, s.loss_pct, s.avg_ms);
              },
      },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  ping.run(ec);
  if (ec) {
    logger.error("Ping {} failed to start: {}", name, ec.message());
  }
}

namespace {
/// Serialize a uint32 as an encapsulated little-endian CDR payload (matches std_msgs/msg/UInt32).
inline std::vector<uint8_t> serialize_uint32(uint32_t value) {
  auto bytes = cdr::serialize<cdr::xcdr1>(value); // CDR_LE with a 4-byte encapsulation header
  if (!bytes) {
    return {};
  }
  const auto *data = reinterpret_cast<const uint8_t *>(bytes->data());
  return std::vector<uint8_t>(data, data + bytes->size());
}

/// Parse a uint32 from an encapsulated CDR payload, or std::nullopt if invalid.
inline std::optional<uint32_t> deserialize_uint32(std::span<const uint8_t> cdr_payload) {
  auto value = cdr::deserialize<uint32_t>(std::as_bytes(cdr_payload));
  if (!value) {
    return std::nullopt;
  }
  return *value;
}
} // namespace

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "ESP32-P4 Function EV Board Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp32 p4 function ev board example]
  auto &board = Board::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);
  logger.info("Display panel: {}", board.get_display_controller_name());

  // Probe the internal I2C bus
  auto &i2c = board.internal_i2c();
  std::vector<uint8_t> found;
  for (uint8_t addr = 1; addr < 128; addr++) {
    if (i2c.probe_device(addr)) {
      found.push_back(addr);
    }
  }
  logger.info("Found {} I2C device(s)", found.size());

  // Display
  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  size_t pixel_buffer_size = board.display_width() * 50;
  if (!board.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // create the GUI: builds the UI (title, status label, rotate / clear
  // buttons, and a transparent circle layer) and starts the task which
  // updates LVGL. All of its public methods are thread-safe, so the touch
  // callback and status task below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});

  // On-screen status state. These are filled in as each subsystem initializes
  // below, and rendered immediately by the status task, so the display shows SD /
  // Ethernet / RTPS coming online live instead of staying blank until the whole
  // bring-up finishes.
  static std::atomic<int> touch_x{0}, touch_y{0}, touch_n{0};
  static std::atomic<bool> sd_card_mounted{false};
  static std::atomic<uint32_t> sd_card_size_mb{0};
  static std::atomic<bool> rtps_running{false}, rtps_has_peers{false};
  static std::atomic<uint32_t> rtps_value{0};
  static int64_t status_start_us = esp_timer_get_time();

  // Status updater: starts now (right after the display is up) and refreshes the
  // on-screen status ~10x/s. Ethernet state is read live from the board; SD and
  // RTPS state are published into the atomics above as those subsystems come up.
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
        std::string rtps_text =
            rtps_running ? ("publishing #" + std::to_string(rtps_value.load()) +
                            (rtps_has_peers ? "" : " (no peers)"))
                         : (board.is_ethernet_connected() ? std::string("not started")
                                                          : std::string("waiting for network"));
        std::string status =
            "Panel:    " + std::string(board.get_display_controller_name()) + " (" +
            std::to_string(board.display_width()) + "x" + std::to_string(board.display_height()) +
            ")\n" + "Touch:    " + std::to_string(touch_n.load()) + " pts (" +
            std::to_string(touch_x.load()) + ", " + std::to_string(touch_y.load()) + ")\n" +
            "SD card:  " +
            (sd_card_mounted ? std::to_string(sd_card_size_mb.load()) + " MB" : "none") + "\n" +
            "Ethernet: " + eth_text + "\n" + "RTPS:     " + rtps_text + "\n" +
            "System:   " + std::to_string(free_internal) + " KB int, " +
            std::to_string(free_psram) + " KB psram free, up " + std::to_string(uptime_s) + " s";
        gui.set_status_text(status);
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 100ms);
        return false;
      },
      .task_config = {.name = "p4-ev status", .stack_size_bytes = 6144}});
  status_task.start();

  // Touch: draw a circle wherever the screen is touched, and play a click on
  // each new touch-down. play_audio() is non-blocking, and the click is gated to
  // the touch-down edge so it doesn't retrigger every poll while held/dragging.
  //
  // The touch task polls at ~16 ms, so we must NOT draw a circle on every poll:
  // a held/stationary finger would stack many translucent (LV_OPA_70) circles at
  // the same point and they'd composite to look fully opaque. Draw only on a new
  // touch-down or once the point has moved at least one radius, so a stationary
  // touch draws a single circle and a drag leaves a spaced trail.
  static constexpr int kCircleRadius = 10;
  // NOTE: this example raises the BSP interrupt- and touch-task stack sizes
  // via sdkconfig.defaults (CONFIG_ESP_P4_EV_BOARD_INTERRUPT_STACK_SIZE /
  // _TOUCH_TASK_STACK_SIZE = 8192); the touch controller is read (polled)
  // from those tasks and their error-logging path needs more than the 4 KB
  // BSP default. See the example README.
  board.initialize_touch([&](const auto &data) {
    auto td = board.touchpad_convert(data);
    static Board::TouchpadData prev_td = {};
    touch_n = td.num_touch_points;
    touch_x = td.x;
    touch_y = td.y;
    if (td.num_touch_points > 0) {
      const bool new_touch = (prev_td != td);
      if (new_touch && !audio_bytes.empty()) {
        board.play_audio(audio_bytes); // non-blocking, touch-down edge only
      }
      if (new_touch && gui.draw_page_active()) {
        gui.draw_circle(td.x, td.y, kCircleRadius);
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

    // Microphone: the ES8311 is full duplex, so the onboard microphone
    // records at the speaker's sample rate. Buffer the recorded mono samples
    // and auto-stop when the buffer is full (the main loop notices and
    // updates the GUI).
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
    if (board.initialize_microphone(mic_callback)) {
      // allocate the recording buffer (16-bit mono at the current sample
      // rate): prefer PSRAM, fall back to a couple of seconds in internal RAM
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
  // the recording (streamed to the speaker by the main loop)
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
      recording_start_us = esp_timer_get_time();
      recording_last_us = recording_start_us.load();
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

  // Ethernet (IP101) — DHCP; the callback fires once an IP is acquired
  static std::atomic<bool> have_ip{false};
  static std::string ip_str{"(no link)"};
  board.initialize_ethernet([&](esp_ip4_addr_t ip) {
    char buf[16];
    esp_ip4addr_ntoa(&ip, buf, sizeof(buf));
    ip_str = buf;
    have_ip = true;
    logger.info("Ethernet IP: {}", ip_str);
  });

  // BOOT button — clears the drawn circles
  //
  // NOTE: GPIO35 is shared with Ethernet RMII TXD1 on this board, so the BOOT
  //       button can't be used as a runtime input while Ethernet is active
  //       (initialize_button() refuses to run when Ethernet is up). It is
  //       disabled here since this example uses Ethernet.
  bool button_initialized = board.initialize_button([&](const auto &event) {
    if (event.active) {
      gui.clear_circles();
    }
  });
  if (button_initialized) {
    logger.error("BOOT button incorrectly initialized while Ethernet is active!");
  } else {
    logger.info("BOOT button not initialized (shared with Ethernet RMII TXD1 pin)");
  }

  // Connectivity self-test: once we have an IP (and a moment for RTPS discovery),
  // ping the gateway and the discovered peer once, then stop. This makes it easy
  // to tell board-vs-network problems apart (e.g. gateway reachable but peer not
  // => client isolation / L2 reachability problem, not the board).
  espp::Task ping_task(
      espp::Task::Config{.callback = [&logger](std::mutex &m, std::condition_variable &cv) -> bool {
                           if (!have_ip) {
                             std::unique_lock<std::mutex> lk(m);
                             cv.wait_for(lk, 250ms);
                             return false; // keep waiting for an IP
                           }
                           // give RTPS discovery a few seconds to find the peer
                           {
                             std::unique_lock<std::mutex> lk(m);
                             cv.wait_for(lk, 4s);
                           }
                           logger.info("=== Connectivity self-test (ping) ===");
                           esp_netif_ip_info_t ip_info{};
                           if (auto *netif = esp_netif_get_default_netif()) {
                             esp_netif_get_ip_info(netif, &ip_info);
                           }
                           char gw[16] = {0};
                           esp_ip4addr_ntoa(&ip_info.gw, gw, sizeof(gw));
                           ping_target(logger, "gateway", gw);
                           std::string peer;
                           {
                             std::lock_guard<std::mutex> lk(g_peer_mutex);
                             peer = g_peer_addr;
                           }
                           if (!peer.empty()) {
                             ping_target(logger, "peer", peer);
                           } else {
                             logger.warn("Ping self-test: no RTPS peer discovered yet to ping");
                           }
                           logger.info("=== Connectivity self-test done ===");
                           return true; // one-shot
                         },
                         .task_config = {.name = "ping-test", .stack_size_bytes = 8192}});
  ping_task.start();
  //! [esp32 p4 function ev board example]

  // Once we have an IP, start an RTPS participant that publishes a counter. The
  // on-screen status is rendered separately by status_task above; this loop drives
  // RTPS and publishes its state into the rtps_* atomics for the status task.
  static bool did_have_ip = false;
  std::shared_ptr<espp::RtpsParticipant> participant = nullptr;
  const std::string topic = "espp/test/counter";
  const std::string rtps_type = "std_msgs::msg::dds_::UInt32_";
  uint32_t value = 0;
  bool published = false;
  static constexpr auto loop_tick = 20ms; // RTPS loop tick
  static constexpr int64_t publish_period_us = 50'000'000;
  int64_t last_publish_us = 0;

  while (true) {
    const int64_t now_us = esp_timer_get_time();

    // (Re)start the RTPS participant when the Ethernet link comes up.
    if (!did_have_ip && have_ip) {
      did_have_ip = true;
      std::string address = ip_str;
      logger.info("Got IP {}, starting RTPS participant", address);
      participant = std::make_shared<espp::RtpsParticipant>(espp::RtpsParticipant::Config{
          .node_name = "espp_publisher",
          .participant_id = 10,
          .advertised_address = address,
          .announce_period = 500ms,
          .on_participant_discovered =
              [&logger](const auto &p) {
                {
                  std::lock_guard<std::mutex> lk(g_peer_mutex);
                  if (g_peer_addr.empty()) {
                    g_peer_addr = p.address;
                  }
                }
                logger.info("discovered participant at {}", p.address);
              },
          .on_endpoint_discovered =
              [&logger](const auto &endpoint) {
                logger.info("discovered {} '{}'", endpoint.is_reader ? "reader" : "writer",
                            endpoint.topic_name);
              },
          .log_level = espp::Logger::Verbosity::DEBUG,
      });
      participant->add_writer({
          .topic_name = topic,
          .type_name = rtps_type,
      });
      if (!participant->start()) {
        logger.error("Failed to start participant (is multicast networking available?)");
        participant.reset();
      }
      value = 0;
      last_publish_us = now_us;
    } else if (did_have_ip && !have_ip) {
      logger.warn("Lost IP, stopping RTPS participant");
      participant.reset();
      did_have_ip = false;
    }
    rtps_running = (participant != nullptr);

    // Publish the next counter value at 2 Hz (independent of the status refresh).
    // Only publish if there is a discovered peer (otherwise the publish() call will return false).
    bool publish_period_elapsed = (now_us - last_publish_us) >= publish_period_us;
    bool can_publish =
        participant && !participant->discovered_participants().empty() && publish_period_elapsed;
    if (can_publish) {
      last_publish_us = now_us;
      published = participant->publish(topic, serialize_uint32(value));
      if (published) {
        logger.info("published {}", value);
        ++value;
      } else {
        logger.warn("publish {} failed (no discovered peers)", value);
      }
    }

    // Publish the RTPS counter/peer state for the status task to render.
    rtps_value = value;
    rtps_has_peers = published;

    // Stream any active playback to the speaker in chunks, advancing by
    // however much the stream buffer accepted
    static size_t play_offset = 0;
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
    // Notice when the recording stopped (button press or buffer full)
    static bool was_recording = false;
    bool now_recording = recording;
    if (was_recording && !now_recording) {
      gui.set_record_active(false);
      gui.set_audio_status(fmt::format("Recorded {:.1f}s ({} plays)",
                                       static_cast<float>(recording_len) /
                                           (board.audio_sample_rate() * sizeof(int16_t)),
                                       LV_SYMBOL_PLAY));
      // report the measured capture rate: samples recorded over the wall
      // clock they took to arrive should match the nominal sample rate
      size_t num_samples = recording_len / sizeof(int16_t);
      float elapsed_s = static_cast<float>(recording_last_us - recording_start_us) / 1e6f;
      float effective_hz = elapsed_s > 0.0f ? num_samples / elapsed_s : 0.0f;
      logger.info("Recorded {} samples in {:.2f} s (~{:.0f} Hz effective, {} Hz nominal)",
                  num_samples, elapsed_s, effective_hz, board.audio_sample_rate());
    }
    was_recording = now_recording;

    std::this_thread::sleep_for(loop_tick);
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
  uint32_t sample_rate = *(reinterpret_cast<const uint32_t *>(&audio_bytes[24]));
  audio_bytes.erase(audio_bytes.begin(), audio_bytes.begin() + 44);
  out_size = audio_bytes.size();
  out_sample_rate = sample_rate;
  return true;
}
