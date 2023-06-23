#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <thread>

#include "sdkconfig.h"

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

#include "rtsp_client.hpp"
#include "rtsp_server.hpp"

#include "jpeg_image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "main", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting RTSP example!");

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  std::string ip_address;
  espp::WifiSta wifi_sta({.ssid = CONFIG_ESP_WIFI_SSID,
                          .password = CONFIG_ESP_WIFI_PASSWORD,
                          .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                          .on_connected = nullptr,
                          .on_disconnected = nullptr,
                          .on_got_ip = [&ip_address](ip_event_got_ip_t *eventdata) {
                            ip_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
                            fmt::print("got IP: {}\n", ip_address);
                          }});

  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(100ms);
  }

  //! [rtsp_server_example]
  const int server_port = CONFIG_RTSP_SERVER_PORT;
  const std::string server_uri = fmt::format("rtsp://{}:{}/mjpeg/1", ip_address, server_port);

  logger.info("Starting RTSP Server on port {}", server_port);
  logger.info("RTSP URI: {}", server_uri);

  espp::RtspServer rtsp_server({
      .server_address = ip_address,
      .port = server_port,
      .path = "/mjpeg/1",
      .log_level = espp::Logger::Verbosity::INFO,
  });
  rtsp_server.start();

  espp::JpegFrame jpeg_frame(jpeg_data, sizeof(jpeg_data));

  logger.info("Parsed JPEG image, num bytes: {}", jpeg_frame.get_data().size());
  logger.info("Created frame of size {}x{}", jpeg_frame.get_width(), jpeg_frame.get_height());
  rtsp_server.send_frame(jpeg_frame);
  //! [rtsp_server_example]

  //! [rtsp_client_example]
  espp::RtspClient rtsp_client({
      .server_address = ip_address, // string of the form {}.{}.{}.{}
      .rtsp_port = CONFIG_RTSP_SERVER_PORT,
      .path = "/mjpeg/1",
      .on_jpeg_frame =
          [](std::unique_ptr<espp::JpegFrame> jpeg_frame) {
            fmt::print("Got JPEG frame of size {}x{}\n", jpeg_frame->get_width(),
                       jpeg_frame->get_height());
          },
      .log_level = espp::Logger::Verbosity::ERROR,
  });

  std::error_code ec;

  do {
    // clear the error code
    ec.clear();
    rtsp_client.connect(ec);
    if (ec) {
      logger.error("Error connecting to server: {}", ec.message());
      logger.info("Retrying in 1s...");
      std::this_thread::sleep_for(1s);
    }
  } while (ec);

  rtsp_client.describe(ec);
  if (ec) {
    logger.error("Error describing server: {}", ec.message());
  }

  rtsp_client.setup(ec);
  if (ec) {
    logger.error("Error setting up server: {}", ec.message());
  }

  rtsp_client.play(ec);
  if (ec) {
    logger.error("Error playing server: {}", ec.message());
  }
  //! [rtsp_client_example]

  // now that both client and server are up, send frames forever
  while (true) {
    rtsp_server.send_frame(jpeg_frame);
    std::this_thread::sleep_for(100ms);
  }
}
