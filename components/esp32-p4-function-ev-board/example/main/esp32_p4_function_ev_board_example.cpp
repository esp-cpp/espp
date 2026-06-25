/**
 * @file esp32_p4_function_ev_board_example.cpp
 * @brief ESP32-P4 Function EV Board BSP example
 *
 * Demonstrates the BSP: MIPI-DSI display + GT911 touch (draw circles and play a
 * click sound wherever you touch), microSD, audio (ES8311), Ethernet (IP101)
 * with an RTPS publisher, and the BOOT button. Shows a live on-screen status
 * read-out (panel, touch, SD, Ethernet, RTPS, and system memory/uptime).
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
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

using namespace std::chrono_literals;
using Board = espp::Esp32P4FunctionEvBoard;

// Address of the most recently discovered RTPS peer (filled by the participant's
// on_participant_discovered callback, read by the ping self-test).
static std::mutex g_peer_mutex;
static std::string g_peer_addr;

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

//////////////////////////////////////////////////////////////////////////////
// Touch-to-draw circle state (a ring buffer of recent touch points)
//////////////////////////////////////////////////////////////////////////////
static constexpr size_t MAX_CIRCLES = 100;
struct Circle {
  int x{0};
  int y{0};
  int radius{0};
  bool visible{false};
};
static std::array<Circle, MAX_CIRCLES> circles;
static size_t next_circle_index = 0;
static size_t visible_circle_count = 0;
static lv_obj_t *circle_layer = nullptr;
static std::recursive_mutex lvgl_mutex;
static std::vector<uint8_t> audio_bytes;

static bool initialize_circle_layer(int width, int height);
static void draw_circle_layer(lv_event_t *event);
static void invalidate_circle_area(const Circle &circle);
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();
static bool load_audio(size_t &out_size, size_t &out_sample_rate);

namespace {
/// Serialize a uint32 as an encapsulated little-endian CDR payload (matches std_msgs/msg/UInt32).
inline std::vector<uint8_t> serialize_uint32(uint32_t value) {
  espp::CdrWriter writer; // defaults: CDR_LE with a 4-byte encapsulation header
  writer.write<uint32_t>(value);
  return writer.take_buffer();
}

/// Parse a uint32 from an encapsulated CDR payload, or std::nullopt if invalid.
inline std::optional<uint32_t> deserialize_uint32(std::span<const uint8_t> cdr) {
  espp::CdrReader reader(cdr);
  uint32_t value = 0;
  if (!reader.valid() || !reader.read<uint32_t>(value)) {
    return std::nullopt;
  }
  return value;
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

  // microSD (optional — only present if a card is inserted)
  bool sd_ok = board.initialize_sdcard({.format_if_mount_failed = false});
  uint32_t sd_size_mb = 0, sd_free_mb = 0;
  if (sd_ok) {
    board.get_sd_card_info(&sd_size_mb, &sd_free_mb);
    logger.info("SD card: {} MB total, {} MB free", sd_size_mb, sd_free_mb);
  } else {
    logger.warn("No SD card mounted");
  }

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
  }

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
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      clear_circles();
    }
  });
  if (button_initialized) {
    logger.error("BOOT button incorrectly initialized while Ethernet is active!");
  } else {
    logger.info("BOOT button not initialized (shared with Ethernet RMII TXD1 pin)");
  }

  // Build the LVGL UI: a title, a status label, a rotate button, and a
  // transparent layer that the touch handler draws circles onto.
  lv_obj_t *bg = nullptr;
  lv_obj_t *title = nullptr;
  static lv_obj_t *status_label = nullptr;
  {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
    bg = lv_obj_create(lv_screen_active());
    lv_obj_set_size(bg, board.display_width(), board.display_height());
    lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

    title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "ESP32-P4 Function EV Board  -  touch to draw!");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    status_label = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 8, 40);

    initialize_circle_layer(board.display_width(), board.display_height());
    lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);
    if (circle_layer) {
      lv_obj_move_foreground(circle_layer);
    }
  }

  // Cycle the display rotation (0 -> 90 -> 180 -> 270) and resize the background
  // and circle layers to match the new orientation. Static so the (non-capturing)
  // button event callback below can call it; it captures app_main locals by
  // reference, which is safe because app_main never returns.
  static auto rotate_display = [&]() {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
    clear_circles();
    static auto rotation = LV_DISPLAY_ROTATION_0;
    rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
    lv_display_set_rotation(lv_display_get_default(), rotation);
    lv_obj_set_size(bg, board.rotated_display_width(), board.rotated_display_height());
    if (circle_layer) {
      lv_obj_set_size(circle_layer, board.rotated_display_width(), board.rotated_display_height());
      lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
      lv_obj_move_foreground(circle_layer);
      lv_obj_invalidate(circle_layer);
    }
    // re-align the labels to the (now reoriented) screen edges
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);
    if (status_label) {
      lv_obj_align(status_label, LV_ALIGN_TOP_LEFT, 8, 40);
    }
  };

  {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
    lv_obj_t *rotate_btn = lv_btn_create(lv_screen_active());
    lv_obj_set_size(rotate_btn, 50, 50);
    lv_obj_align(rotate_btn, LV_ALIGN_TOP_RIGHT, -8, 8);
    lv_obj_t *btn_label = lv_label_create(rotate_btn);
    lv_label_set_text(btn_label, LV_SYMBOL_REFRESH);
    lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(
        rotate_btn, [](lv_event_t *) { rotate_display(); }, LV_EVENT_CLICKED, nullptr);
  }

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
  static std::atomic<int> touch_x{0}, touch_y{0}, touch_n{0};
  board.initialize_touch([&](const auto &data) {
    static int prev_touch_n = 0;
    static int last_drawn_x = 0, last_drawn_y = 0;
    auto td = board.touchpad_convert(data);
    touch_n = td.num_touch_points;
    touch_x = td.x;
    touch_y = td.y;
    if (td.num_touch_points > 0) {
      const bool new_touch = (prev_touch_n == 0);
      if (new_touch && !audio_bytes.empty()) {
        board.play_audio(audio_bytes); // non-blocking, touch-down edge only
      }
      const int dx = static_cast<int>(td.x) - last_drawn_x;
      const int dy = static_cast<int>(td.y) - last_drawn_y;
      const bool moved = (dx * dx + dy * dy) >= (kCircleRadius * kCircleRadius);
      if (new_touch || moved) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(td.x, td.y, kCircleRadius);
        last_drawn_x = td.x;
        last_drawn_y = td.y;
      }
    }
    prev_touch_n = td.num_touch_points;
  });

  // Run the LVGL task handler periodically
  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        {
                          std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
                          lv_task_handler();
                        }
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {.name = "lvgl", .stack_size_bytes = 8192}});
  lv_task.start();

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

  // Once we have an IP, start an RTPS participant that publishes a counter.
  // The display status is refreshed at 50 Hz while RTPS publishes at 2 Hz.
  static bool did_have_ip = false;
  std::shared_ptr<espp::RtpsParticipant> participant = nullptr;
  const std::string topic = "espp/test/counter";
  const std::string rtps_type = "std_msgs::msg::dds_::UInt32_";
  uint32_t value = 0;
  bool published = false;
  const int64_t start_us = esp_timer_get_time();
  static constexpr auto status_period = 20ms;           // 50 Hz display status update
  static constexpr int64_t publish_period_us = 500'000; // 2 Hz RTPS publish
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

    // Build and show the on-screen status at 50 Hz.
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    int uptime_s = static_cast<int>((now_us - start_us) / 1'000'000);
    std::string rtps_text =
        participant ? ("publishing '" + topic + "' #" + std::to_string(value) +
                       (published ? "" : " (no peers)"))
                    : (have_ip ? std::string("not started") : std::string("waiting for network"));
    std::string status =
        "Panel:    " + std::string(board.get_display_controller_name()) + " (" +
        std::to_string(board.display_width()) + "x" + std::to_string(board.display_height()) +
        ")\n" + "Touch:    " + std::to_string(touch_n.load()) + " pts (" +
        std::to_string(touch_x.load()) + ", " + std::to_string(touch_y.load()) + ")\n" +
        "SD card:  " + (sd_ok ? std::to_string(sd_size_mb) + " MB" : "none") + "\n" +
        "Ethernet: " + (have_ip ? ip_str : std::string("(no link)")) + "\n" +
        "RTPS:     " + rtps_text + "\n" + "System:   " + std::to_string(free_internal) +
        " KB int, " + std::to_string(free_psram) + " KB psram free, up " +
        std::to_string(uptime_s) + " s";

    {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      if (status_label) {
        lv_label_set_text(status_label, status.c_str());
      }
    }

    std::this_thread::sleep_for(status_period);
  }
}

//////////////////////////////////////////////////////////////////////////////
// LVGL circle-drawing helpers (a transparent full-screen layer with a custom
// draw callback that renders the ring buffer of recent touch points).
//////////////////////////////////////////////////////////////////////////////
static bool initialize_circle_layer(int width, int height) {
  if (circle_layer) {
    return true;
  }
  circle_layer = lv_obj_create(lv_screen_active());
  if (!circle_layer) {
    return false;
  }
  lv_obj_remove_style_all(circle_layer);
  lv_obj_set_size(circle_layer, width, height);
  lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(circle_layer, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(circle_layer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(circle_layer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(circle_layer, 0, 0);
  lv_obj_set_style_outline_width(circle_layer, 0, 0);
  lv_obj_set_style_shadow_width(circle_layer, 0, 0);
  lv_obj_add_event_cb(circle_layer, draw_circle_layer, LV_EVENT_DRAW_MAIN, nullptr);
  return true;
}

static void draw_circle_layer(lv_event_t *event) {
  if (visible_circle_count == 0) {
    return;
  }
  auto *obj = static_cast<lv_obj_t *>(lv_event_get_current_target(event));
  auto *layer = lv_event_get_layer(event);
  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);

  lv_draw_rect_dsc_t rect_dsc;
  lv_draw_rect_dsc_init(&rect_dsc);
  rect_dsc.base.layer = layer;
  rect_dsc.radius = LV_RADIUS_CIRCLE;
  rect_dsc.bg_opa = LV_OPA_70;
  rect_dsc.bg_color = lv_color_make(0, 255, 255);
  rect_dsc.border_width = 0;
  rect_dsc.outline_width = 0;
  rect_dsc.shadow_width = 0;

  for (const auto &circle : circles) {
    if (!circle.visible) {
      continue;
    }
    lv_area_t coords = {
        .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
        .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
        .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
        .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
    };
    lv_draw_rect(layer, &rect_dsc, &coords);
  }
}

static void invalidate_circle_area(const Circle &circle) {
  if (!circle_layer || circle.radius <= 0) {
    return;
  }
  lv_area_t obj_coords;
  lv_obj_get_coords(circle_layer, &obj_coords);
  lv_area_t coords = {
      .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
      .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
      .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
      .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
  };
  lv_obj_invalidate_area(circle_layer, &coords);
}

static void draw_circle(int x0, int y0, int radius) {
  if (!circle_layer) {
    return;
  }
  lv_obj_move_foreground(circle_layer);
  Circle previous_circle = circles[next_circle_index];
  circles[next_circle_index] = {.x = x0, .y = y0, .radius = radius, .visible = true};
  next_circle_index = (next_circle_index + 1) % circles.size();
  if (visible_circle_count < circles.size()) {
    visible_circle_count++;
  }
  if (previous_circle.visible) {
    invalidate_circle_area(previous_circle);
  }
  invalidate_circle_area(circles[(next_circle_index + circles.size() - 1) % circles.size()]);
}

static void clear_circles() {
  for (auto &circle : circles) {
    if (circle.visible) {
      invalidate_circle_area(circle);
    }
    circle.visible = false;
  }
  next_circle_index = 0;
  visible_circle_count = 0;
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
