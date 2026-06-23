#include "sdkconfig.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string_view>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/i2s_common.h>
#include <driver/i2s_pdm.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <hal/i2c_types.h>
#include <mdns.h>

#include "cli.hpp"
#include "generic_packetizer.hpp"
#include "heap_monitor.hpp"
#include "rtsp_server.hpp"
#include "task.hpp"
#include "task_monitor.hpp"
#include "wifi_sta.hpp"
#include "wifi_sta_menu.hpp"
#include "xiao-esp32s3-sense.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "XIAO Camera Streamer", .level = espp::Logger::Verbosity::INFO});

namespace {
constexpr auto idle_capture_poll_period = 250ms;
constexpr auto dma_pressure_backoff = 250ms;
constexpr auto camera_capture_error_backoff = 100ms;
constexpr auto target_video_period = 100ms;
constexpr auto audio_capture_timeout = 50ms;
constexpr size_t min_dma_free_bytes_for_streaming = 12 * 1024;
constexpr size_t min_dma_largest_block_for_streaming = 4 * 1024;

constexpr int video_track_id = 0;
constexpr int audio_track_id = 1;
constexpr uint32_t microphone_sample_rate_hz =
    espp::XiaoEsp32S3Sense::microphone_default_sample_rate_hz();
constexpr size_t microphone_frame_samples = microphone_sample_rate_hz / 50; // 20 ms
constexpr size_t microphone_frame_bytes = microphone_frame_samples * sizeof(int16_t);
constexpr char rtsp_path[] = "mjpeg/1";

template <typename Clock, typename Duration>
bool wait_until_or_stop(std::mutex &m, std::condition_variable &cv, bool &task_notified,
                        const std::chrono::time_point<Clock, Duration> &deadline) {
  std::unique_lock<std::mutex> lk(m);
  auto stop_requested = cv.wait_until(lk, deadline, [&task_notified] { return task_notified; });
  task_notified = false;
  return stop_requested;
}
} // namespace

std::recursive_mutex server_mutex;
std::unique_ptr<espp::Task> camera_task;
std::unique_ptr<espp::Task> audio_task;
std::unique_ptr<espp::Task> memory_monitor_task;
std::unique_ptr<espp::TaskMonitor> task_monitor;
std::shared_ptr<espp::RtspServer> rtsp_server;
i2s_chan_handle_t microphone_rx_channel{nullptr};
bool mdns_started{false};
std::atomic<size_t> video_frames_streamed{0};
std::atomic<size_t> audio_frames_streamed{0};

esp_err_t initialize_camera(void);
esp_err_t initialize_microphone(void);
void deinitialize_microphone(void);
bool start_rtsp_server(std::string_view server_address, int server_port);
void stop_streaming(void);
bool initialize_and_test_sdcard(void);
bool camera_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified);
bool audio_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified);
bool memory_monitor_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified);

extern "C" void app_main(void) {
  logger.info("Bootup");

  static constexpr float disconnected_led_breathing_period = 1.0f;
  static constexpr float connected_led_breathing_period = 3.5f;

  //! [xiao esp32s3 sense example]
  auto &sense = espp::XiaoEsp32S3Sense::get();
  auto microphone = sense.microphone_pins();
  auto sd_card = sense.sd_card_pins();
  logger.info("Using XIAO ESP32S3 Sense camera + microphone");
  logger.info("User LED pin: {}", static_cast<int>(sense.user_led_pin()));
  logger.info("Microphone pins: clk={}, data={}", static_cast<int>(microphone.clk),
              static_cast<int>(microphone.data));
  logger.info("microSD pins: clk={}, mosi={}, miso={}, cs={}", static_cast<int>(sd_card.clk),
              static_cast<int>(sd_card.mosi), static_cast<int>(sd_card.miso),
              static_cast<int>(sd_card.cs));

  if (!sense.initialize_led(disconnected_led_breathing_period)) {
    logger.error("Could not initialize user LED");
    return;
  }
  sense.start_led_breathing();

  initialize_and_test_sdcard();

  logger.info("Initializing camera");
  auto err = initialize_camera();
  if (err != ESP_OK) {
    logger.error("Could not initialize camera: {} '{}'", err, esp_err_to_name(err));
    return;
  }

  logger.info("Initializing microphone");
  err = initialize_microphone();
  if (err != ESP_OK) {
    logger.error("Could not initialize microphone: {} '{}'", err, esp_err_to_name(err));
    deinitialize_microphone();
    return;
  }
  //! [xiao esp32s3 sense example]

  logger.info("Starting memory monitors");
  task_monitor = std::make_unique<espp::TaskMonitor>(espp::TaskMonitor::Config{.period = 30s});
  memory_monitor_task = espp::Task::make_unique(espp::Task::Config{
      .callback = memory_monitor_task_fn,
      .task_config =
          {
              .name = "Memory Monitor",
              .stack_size_bytes = 6 * 1024,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  logger.info("Initializing WiFi");
  espp::WifiSta wifi_sta(
      {.ssid = CONFIG_ESP_WIFI_SSID,
       .password = CONFIG_ESP_WIFI_PASSWORD,
       .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
       .on_connected =
           []() {
             auto &sense = espp::XiaoEsp32S3Sense::get();
             logger.info("Connected to WiFi, waiting for IP address");
             sense.set_led_breathing_period(connected_led_breathing_period);
             memory_monitor_task->start();
           },
       .on_disconnected =
           []() {
             auto &sense = espp::XiaoEsp32S3Sense::get();
             logger.info("WiFi disconnected, stopping RTSP streaming");
             sense.set_led_breathing_period(disconnected_led_breathing_period);
             stop_streaming();
             memory_monitor_task->stop();
           },
       .on_got_ip =
           [](ip_event_got_ip_t *eventdata) {
             auto server_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
             logger.info("got IP: {}", server_address);
             stop_streaming();

             if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) {
               logger.warn(
                   "Could not disable WiFi power save; RTSP streaming may hit TX backpressure");
             } else {
               logger.info("Disabled WiFi power save for RTSP streaming");
             }

             if (!start_rtsp_server(server_address, CONFIG_RTSP_SERVER_PORT)) {
               logger.error("RTSP server failed to start, not starting capture tasks");
               return;
             }

             auto new_camera_task = espp::Task::make_unique(espp::Task::Config{
                 .callback = camera_task_fn,
                 .task_config = {.name = "Camera Task", .priority = 10},
                 .log_level = espp::Logger::Verbosity::WARN,
             });
             if (!new_camera_task->start()) {
               logger.error("Could not start camera task");
               stop_streaming();
               return;
             }

             auto new_audio_task = espp::Task::make_unique(espp::Task::Config{
                 .callback = audio_task_fn,
                 .task_config =
                     {
                         .name = "Audio Task",
                         .stack_size_bytes = 6 * 1024,
                         .priority = 9,
                     },
                 .log_level = espp::Logger::Verbosity::WARN,
             });
             if (!new_audio_task->start()) {
               logger.error("Could not start audio task");
               new_camera_task.reset();
               stop_streaming();
               return;
             }

             std::lock_guard<std::recursive_mutex> lock(server_mutex);
             camera_task = std::move(new_camera_task);
             audio_task = std::move(new_audio_task);
           }});

  espp::WifiStaMenu sta_menu(wifi_sta);
  auto root_menu = sta_menu.get();
  root_menu->Insert(
      "memory",
      [](std::ostream &out) {
        out << "Video frames streamed: " << video_frames_streamed.load() << '\n';
        out << "Audio frames streamed: " << audio_frames_streamed.load() << '\n';
        out << espp::HeapMonitor::get_table(
                   {MALLOC_CAP_DEFAULT, MALLOC_CAP_INTERNAL, MALLOC_CAP_SPIRAM, MALLOC_CAP_DMA})
            << std::endl;
      },
      "Display current heap monitor information.");

  cli::Cli cli(std::move(root_menu));
  cli::SetColor();
  cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

  espp::Cli input(cli);
  input.SetInputHistorySize(10);
  input.Start();
}

esp_err_t initialize_camera(void) {
  auto &sense = espp::XiaoEsp32S3Sense::get();
  auto camera = sense.camera_pins();

  static camera_config_t camera_config = {
      .pin_pwdn = static_cast<int>(camera.pwdn),
      .pin_reset = static_cast<int>(camera.reset),
      .pin_xclk = static_cast<int>(camera.xclk),
      .pin_sccb_sda = static_cast<int>(camera.sccb_sda),
      .pin_sccb_scl = static_cast<int>(camera.sccb_scl),

      .pin_d7 = static_cast<int>(camera.d7),
      .pin_d6 = static_cast<int>(camera.d6),
      .pin_d5 = static_cast<int>(camera.d5),
      .pin_d4 = static_cast<int>(camera.d4),
      .pin_d3 = static_cast<int>(camera.d3),
      .pin_d2 = static_cast<int>(camera.d2),
      .pin_d1 = static_cast<int>(camera.d1),
      .pin_d0 = static_cast<int>(camera.d0),
      .pin_vsync = static_cast<int>(camera.vsync),
      .pin_href = static_cast<int>(camera.href),
      .pin_pclk = static_cast<int>(camera.pclk),

      .xclk_freq_hz = sense.camera_xclk_freq_hz(),
      .ledc_timer = LEDC_TIMER_0,
      .ledc_channel = LEDC_CHANNEL_0,

      .pixel_format = PIXFORMAT_JPEG,
      .frame_size = FRAMESIZE_QVGA,
      .jpeg_quality = 15,
      .fb_count = 2,
      .fb_location = CAMERA_FB_IN_PSRAM,
      .grab_mode = CAMERA_GRAB_LATEST,
      .sccb_i2c_port = I2C_NUM_0,
  };

  auto err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    return err;
  }

  if (auto *sensor = esp_camera_sensor_get(); sensor != nullptr) {
    sensor->set_vflip(sensor, false);
    sensor->set_hmirror(sensor, false);
  }
  return ESP_OK;
}

esp_err_t initialize_microphone(void) {
  if (microphone_rx_channel != nullptr) {
    return ESP_OK;
  }

  auto &sense = espp::XiaoEsp32S3Sense::get();
  i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  channel_config.auto_clear = true;
  channel_config.dma_desc_num = 4;
  channel_config.dma_frame_num = microphone_frame_samples;

  auto err = i2s_new_channel(&channel_config, nullptr, &microphone_rx_channel);
  if (err != ESP_OK) {
    logger.error("Could not allocate I2S RX channel: {} '{}'", err, esp_err_to_name(err));
    microphone_rx_channel = nullptr;
    return err;
  }

  auto microphone_config = sense.microphone_config(microphone_sample_rate_hz);
  err = i2s_channel_init_pdm_rx_mode(microphone_rx_channel, &microphone_config);
  if (err != ESP_OK) {
    logger.error("Could not initialize PDM RX mode: {} '{}'", err, esp_err_to_name(err));
    deinitialize_microphone();
    return err;
  }

  err = i2s_channel_enable(microphone_rx_channel);
  if (err != ESP_OK) {
    logger.error("Could not enable microphone channel: {} '{}'", err, esp_err_to_name(err));
    deinitialize_microphone();
    return err;
  }

  logger.info("Microphone initialized for {} Hz mono PCM", microphone_sample_rate_hz);
  return ESP_OK;
}

void deinitialize_microphone(void) {
  if (microphone_rx_channel == nullptr) {
    return;
  }
  i2s_channel_disable(microphone_rx_channel);
  i2s_del_channel(microphone_rx_channel);
  microphone_rx_channel = nullptr;
}

bool initialize_and_test_sdcard(void) {
  auto &sense = espp::XiaoEsp32S3Sense::get();
  if (!sense.initialize_sdcard()) {
    logger.warn("Could not initialize microSD card, continuing without storage");
    return false;
  }

  constexpr char test_file_path[] = "/sdcard/xiao_sdcard_smoke_test.txt";
  constexpr std::string_view test_payload = "xiao-esp32s3-sense sdcard smoke test\n";

  auto *file = std::fopen(test_file_path, "wb");
  if (file == nullptr) {
    logger.error("Could not open '{}' for writing", test_file_path);
    return false;
  }

  size_t bytes_written = std::fwrite(test_payload.data(), 1, test_payload.size(), file);
  std::fclose(file);
  if (bytes_written != test_payload.size()) {
    logger.error("microSD smoke test short write: wrote {} of {} bytes", bytes_written,
                 test_payload.size());
    std::remove(test_file_path);
    return false;
  }

  std::array<char, 128> readback{};
  file = std::fopen(test_file_path, "rb");
  if (file == nullptr) {
    logger.error("Could not reopen '{}' for reading", test_file_path);
    std::remove(test_file_path);
    return false;
  }

  size_t bytes_read = std::fread(readback.data(), 1, readback.size() - 1, file);
  std::fclose(file);
  std::remove(test_file_path);

  std::string_view readback_view(readback.data(), bytes_read);
  if (readback_view != test_payload) {
    logger.error("microSD smoke test readback mismatch: expected '{}', got '{}'", test_payload,
                 readback_view);
    return false;
  }

  logger.info("microSD smoke test passed using '{}'", test_file_path);
  return true;
}

bool start_rtsp_server(std::string_view server_address, int server_port) {
  logger.info("Creating RTSP server at {}:{}", server_address, server_port);

  auto server = std::make_shared<espp::RtspServer>(espp::RtspServer::Config{
      .server_address = std::string(server_address),
      .port = server_port,
      .path = rtsp_path,
      .log_level = espp::Logger::Verbosity::WARN,
      .accept_task_stack_size_bytes = CONFIG_RTSP_ACCEPT_TASK_STACK_SIZE,
      .session_task_stack_size_bytes = CONFIG_RTSP_SESSION_TASK_STACK_SIZE,
      .control_task_stack_size_bytes = CONFIG_RTSP_CONTROL_TASK_STACK_SIZE,
  });
  server->set_session_log_level(espp::Logger::Verbosity::WARN);

  auto video_packetizer = std::make_shared<espp::MjpegPacketizer>(
      espp::MjpegPacketizer::Config{.max_payload_size = 1400});
  server->add_track(
      espp::RtspServer::TrackConfig{.track_id = video_track_id, .packetizer = video_packetizer});

  auto audio_packetizer = std::make_shared<espp::GenericPacketizer>(espp::GenericPacketizer::Config{
      .max_payload_size = 1400,
      .payload_type = 97,
      .clock_rate = microphone_sample_rate_hz,
      .encoding_name = "L16",
      .channels = 1,
      .fmtp = "",
      .media_type = espp::MediaType::AUDIO,
      .log_level = espp::Logger::Verbosity::WARN,
  });
  server->add_track(
      espp::RtspServer::TrackConfig{.track_id = audio_track_id, .packetizer = audio_packetizer});

  if (!server->start()) {
    logger.error("Failed to start RTSP server on {}:{}", server_address, server_port);
    return false;
  }

  logger.info("Initializing mDNS");
  auto err = mdns_init();
  if (err != ESP_OK) {
    logger.error("Could not initialize mDNS: {}", err);
    server.reset();
    return false;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  std::string hostname = fmt::format("xiao-camera-streamer-{:x}{:x}{:x}", mac[3], mac[4], mac[5]);
  err = mdns_hostname_set(hostname.c_str());
  if (err != ESP_OK) {
    logger.error("Could not set mDNS hostname: {}", err);
    mdns_free();
    server.reset();
    return false;
  }
  logger.info("mDNS hostname set to '{}'", hostname);

  err = mdns_instance_name_set("XIAO Camera Streamer");
  if (err != ESP_OK) {
    logger.error("Could not set mDNS instance name: {}", err);
    mdns_free();
    server.reset();
    return false;
  }

  err = mdns_service_add("RTSP Server", "_rtsp", "_tcp", server_port, nullptr, 0);
  if (err != ESP_OK) {
    logger.error("Could not add mDNS service: {}", err);
    mdns_free();
    server.reset();
    return false;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    rtsp_server = std::move(server);
    mdns_started = true;
  }

  logger.info("mDNS initialized");
  return true;
}

void stop_streaming(void) {
  std::unique_ptr<espp::Task> old_camera_task;
  std::unique_ptr<espp::Task> old_audio_task;
  std::shared_ptr<espp::RtspServer> old_rtsp_server;
  bool should_stop_mdns = false;

  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    old_camera_task = std::move(camera_task);
    old_audio_task = std::move(audio_task);
    old_rtsp_server = std::move(rtsp_server);
    should_stop_mdns = mdns_started;
    mdns_started = false;
  }

  old_audio_task.reset();
  old_camera_task.reset();

  if (should_stop_mdns) {
    logger.info("Deiniting MDNS");
    mdns_free();
  }

  old_rtsp_server.reset();
}

bool camera_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  auto start = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    if (!rtsp_server || !rtsp_server->has_active_sessions()) {
      return wait_until_or_stop(m, cv, task_notified, start + idle_capture_poll_period);
    }
    auto recommended_capture_period = rtsp_server->get_recommended_capture_period();
    auto capture_cooldown = rtsp_server->get_capture_cooldown();
    if (capture_cooldown > 0ms) {
      return wait_until_or_stop(m, cv, task_notified,
                                start + std::max(recommended_capture_period, capture_cooldown));
    }
  }

  auto dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
  auto dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  if (dma_free < min_dma_free_bytes_for_streaming ||
      dma_largest < min_dma_largest_block_for_streaming) {
    static auto last_pressure_log = std::chrono::steady_clock::time_point{};
    auto now = std::chrono::steady_clock::now();
    if (now - last_pressure_log >= 2s) {
      logger.warn("Pausing capture to recover DMA pressure: free={} B, largest_block={} B",
                  dma_free, dma_largest);
      last_pressure_log = now;
    }
    return wait_until_or_stop(m, cv, task_notified, start + dma_pressure_backoff);
  }

  auto *frame_buffer = esp_camera_fb_get();
  if (frame_buffer == nullptr) {
    logger.error("Camera capture failed");
    return wait_until_or_stop(m, cv, task_notified,
                              std::chrono::steady_clock::now() + camera_capture_error_backoff);
  }

  if (frame_buffer->len < 2 || frame_buffer->buf[frame_buffer->len - 2] != 0xFF ||
      frame_buffer->buf[frame_buffer->len - 1] != 0xD9) {
    static auto last_bad_frame_log = std::chrono::steady_clock::time_point{};
    auto now = std::chrono::steady_clock::now();
    if (now - last_bad_frame_log >= 2s) {
      logger.warn("Dropping camera frame with invalid JPEG end marker");
      last_bad_frame_log = now;
    }
    esp_camera_fb_return(frame_buffer);
    return wait_until_or_stop(m, cv, task_notified,
                              std::chrono::steady_clock::now() + camera_capture_error_backoff);
  }

  std::span<const uint8_t> jpg_buf(frame_buffer->buf, frame_buffer->len);
  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    if (rtsp_server) {
      rtsp_server->send_frame(video_track_id, jpg_buf);
      video_frames_streamed++;
    }
  }

  esp_camera_fb_return(frame_buffer);

  auto capture_period = target_video_period;
  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    if (rtsp_server) {
      capture_period = std::max(capture_period, rtsp_server->get_recommended_capture_period());
    }
  }
  return wait_until_or_stop(m, cv, task_notified, start + capture_period);
}

bool audio_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  auto start = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    if (!rtsp_server || !rtsp_server->has_active_sessions()) {
      return wait_until_or_stop(m, cv, task_notified, start + idle_capture_poll_period);
    }
    auto capture_cooldown = rtsp_server->get_capture_cooldown();
    if (capture_cooldown > 0ms) {
      return wait_until_or_stop(m, cv, task_notified, start + capture_cooldown);
    }
  }

  if (microphone_rx_channel == nullptr) {
    logger.error("Microphone channel is not initialized");
    return wait_until_or_stop(m, cv, task_notified, start + idle_capture_poll_period);
  }

  std::array<int16_t, microphone_frame_samples> audio_samples{};
  size_t bytes_read = 0;
  auto err = i2s_channel_read(microphone_rx_channel, audio_samples.data(), microphone_frame_bytes,
                              &bytes_read, audio_capture_timeout.count());
  if (err == ESP_ERR_TIMEOUT) {
    return false;
  }
  if (err != ESP_OK) {
    logger.error("Failed reading microphone samples: {} '{}'", err, esp_err_to_name(err));
    return wait_until_or_stop(m, cv, task_notified, start + idle_capture_poll_period);
  }
  if (bytes_read == 0) {
    return false;
  }

  auto *audio_bytes = reinterpret_cast<const uint8_t *>(audio_samples.data());
  {
    std::lock_guard<std::recursive_mutex> lock(server_mutex);
    if (rtsp_server) {
      std::span<const uint8_t> audio_frame(audio_bytes, bytes_read);
      rtsp_server->send_frame(audio_track_id, audio_frame);
      audio_frames_streamed++;
    }
  }
  return false;
}

bool memory_monitor_task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  auto start = std::chrono::steady_clock::now();
  static size_t last_video_frames = 0;
  static size_t last_audio_frames = 0;

  auto total_video_frames = video_frames_streamed.load();
  auto total_audio_frames = audio_frames_streamed.load();
  auto delta_video_frames = total_video_frames - last_video_frames;
  auto delta_audio_frames = total_audio_frames - last_audio_frames;
  last_video_frames = total_video_frames;
  last_audio_frames = total_audio_frames;

  logger.info("Frames streamed: video={} (+{}), audio={} (+{})\n{}", total_video_frames,
              delta_video_frames, total_audio_frames, delta_audio_frames,
              espp::HeapMonitor::get_table(
                  {MALLOC_CAP_DEFAULT, MALLOC_CAP_INTERNAL, MALLOC_CAP_SPIRAM, MALLOC_CAP_DMA}));

  return wait_until_or_stop(m, cv, task_notified, start + 10s);
}
