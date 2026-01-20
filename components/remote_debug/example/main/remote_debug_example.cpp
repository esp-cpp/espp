#include <chrono>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "nvs.hpp"
#include "remote_debug.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Remote Debug Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting Remote Debug Example");

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  //! [remote debug example]

  // Connect to WiFi using espp WifiSta
  logger.info("Connecting to WiFi SSID: {}", CONFIG_REMOTE_DEBUG_WIFI_SSID);

  espp::WifiSta wifi_sta({.ssid = CONFIG_REMOTE_DEBUG_WIFI_SSID,
                          .password = CONFIG_REMOTE_DEBUG_WIFI_PASSWORD,
                          .num_connect_retries = 5,
                          .on_connected = [&logger]() { logger.info("WiFi connected!"); },
                          .on_disconnected = [&logger]() { logger.warn("WiFi disconnected!"); },
                          .on_got_ip =
                              [&logger](ip_event_got_ip_t *event) {
                                logger.info("got IP: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
                              }});

  // Wait for connection
  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  logger.info("WiFi connected successfully");

  // Build GPIO list from menuconfig
  std::vector<espp::RemoteDebug::GpioConfig> gpios;
#if CONFIG_REMOTE_DEBUG_NUM_GPIOS >= 1
  gpios.push_back(
      {.pin = static_cast<gpio_num_t>(CONFIG_REMOTE_DEBUG_GPIO_0), .mode = GPIO_MODE_OUTPUT});
#endif
#if CONFIG_REMOTE_DEBUG_NUM_GPIOS >= 2
  gpios.push_back(
      {.pin = static_cast<gpio_num_t>(CONFIG_REMOTE_DEBUG_GPIO_1), .mode = GPIO_MODE_OUTPUT});
#endif
#if CONFIG_REMOTE_DEBUG_NUM_GPIOS >= 3
  gpios.push_back(
      {.pin = static_cast<gpio_num_t>(CONFIG_REMOTE_DEBUG_GPIO_2), .mode = GPIO_MODE_OUTPUT});
#endif
#if CONFIG_REMOTE_DEBUG_NUM_GPIOS >= 4
  gpios.push_back(
      {.pin = static_cast<gpio_num_t>(CONFIG_REMOTE_DEBUG_GPIO_3), .mode = GPIO_MODE_OUTPUT});
#endif

  // Build ADC list from menuconfig
#if CONFIG_REMOTE_DEBUG_NUM_ADCS >= 1
  int adc_sample_rate_hz = CONFIG_REMOTE_DEBUG_ADC_SAMPLE_RATE_HZ;
  size_t adc_buffer_size = CONFIG_REMOTE_DEBUG_ADC_BUFFER_SIZE;
#else
  int adc_sample_rate_hz = 1; // Default to 1 Hz if no ADCs configured
  size_t adc_buffer_size = 1; // Default to buffer size of 1 if no ADCs configured
#endif

  std::vector<espp::RemoteDebug::AdcConfig> adcs;
#if CONFIG_REMOTE_DEBUG_NUM_ADCS >= 1
  adcs.push_back(
      {.unit = ADC_UNIT_1, .channel = static_cast<adc_channel_t>(CONFIG_REMOTE_DEBUG_ADC_0)});
#endif
#if CONFIG_REMOTE_DEBUG_NUM_ADCS >= 2
  adcs.push_back(
      {.unit = ADC_UNIT_1, .channel = static_cast<adc_channel_t>(CONFIG_REMOTE_DEBUG_ADC_1)});
#endif
#if CONFIG_REMOTE_DEBUG_NUM_ADCS >= 3
  adcs.push_back(
      {.unit = ADC_UNIT_1, .channel = static_cast<adc_channel_t>(CONFIG_REMOTE_DEBUG_ADC_2)});
#endif
#if CONFIG_REMOTE_DEBUG_NUM_ADCS >= 4
  adcs.push_back(
      {.unit = ADC_UNIT_1, .channel = static_cast<adc_channel_t>(CONFIG_REMOTE_DEBUG_ADC_3)});
#endif

  // Configure remote debug
  espp::RemoteDebug::Config config{
      .gpios = gpios,
      .adcs = adcs,
      .server_port = static_cast<uint16_t>(CONFIG_REMOTE_DEBUG_SERVER_PORT),
      .adc_sample_rate = std::chrono::milliseconds(1000 / adc_sample_rate_hz),
      .adc_history_size = adc_buffer_size,
      .log_level = espp::Logger::Verbosity::INFO};

  espp::RemoteDebug remote_debug(config);
  remote_debug.start();

  logger.info("Remote Debug Server started on port {}!", CONFIG_REMOTE_DEBUG_SERVER_PORT);
  logger.info("GPIO pins available: {}", gpios.size());
  logger.info("ADC channels available: {}", adcs.size());

  // Keep running
  while (true) {
    std::this_thread::sleep_for(1s);
  }

  //! [remote debug example]
}
