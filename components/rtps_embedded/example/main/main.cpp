
#include <cstdio>
#include <cstring>
#include <string>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rtps/entities/Domain.h"
#include "logger.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

namespace {

static const char *TAG = "rtps_example";
static bool s_started = false;
static rtps::Domain *s_domain = nullptr;
static rtps::Participant *s_participant = nullptr;
static rtps::Writer *s_writer = nullptr;
static rtps::Reader *s_reader = nullptr;

bool send_text_message(const char *text) {
  if (text == nullptr || s_writer == nullptr) {
    return false;
  }

  const size_t len = strnlen(text, 127);
  const rtps::CacheChange *change = s_writer->newChange(
      rtps::ChangeKind_t::ALIVE, reinterpret_cast<const uint8_t *>(text),
      static_cast<rtps::DataSize_t>(len + 1));

  return change != nullptr;
}

void reader_cb(void * /*callee*/, const rtps::ReaderCacheChange &change) {
  char buffer[128] = {0};
  const rtps::DataSize_t copy_len =
      (change.getDataSize() < static_cast<rtps::DataSize_t>(sizeof(buffer) - 1))
          ? change.getDataSize()
          : static_cast<rtps::DataSize_t>(sizeof(buffer) - 1);

  if (copy_len == 0 ||
      !change.copyInto(reinterpret_cast<uint8_t *>(buffer), sizeof(buffer))) {
    return;
  }

  ESP_LOGI(TAG, "rx (%u B): %s", static_cast<unsigned>(copy_len), buffer);

#if CONFIG_RTPS_EXAMPLE_ROLE_RESPONDER
  // Echo the message back on the publish topic.
  if (!send_text_message(buffer)) {
    ESP_LOGW(TAG, "tx echo dropped (history full or no matched reader)");
  }
#endif
}

#if CONFIG_RTPS_EXAMPLE_ROLE_INITIATOR
void publisher_task(void * /*arg*/) {
  uint32_t counter = 0;
  while (true) {
    char msg[32];
    snprintf(msg, sizeof(msg), "request %u", static_cast<unsigned>(counter++));
    if (!send_text_message(msg)) {
      ESP_LOGW(TAG, "tx dropped (history full or no matched reader)");
    } else {
      ESP_LOGI(TAG, "tx: %s", msg);
    }
    vTaskDelay(pdMS_TO_TICKS(CONFIG_RTPS_EXAMPLE_PUBLISH_PERIOD_MS));
  }
}
#endif

} // namespace

// Start the RTPS stack. Topics are derived from the configured prefix:
//   initiator publishes on <prefix>/request, subscribes to <prefix>/response
//   responder  publishes on <prefix>/response, subscribes to <prefix>/request
extern "C" void embedded_rtps_start(const rtps::Ip4AddressBytes &local_ip) {
  if (s_started) {
    return;
  }

  const std::string prefix = CONFIG_RTPS_EXAMPLE_TOPIC_PREFIX;
#if CONFIG_RTPS_EXAMPLE_ROLE_INITIATOR
  const std::string pub_topic = prefix + "/request";
  const std::string sub_topic = prefix + "/response";
#else
  const std::string pub_topic = prefix + "/response";
  const std::string sub_topic = prefix + "/request";
#endif

  static rtps::Domain domain(local_ip);
  s_domain = &domain;

  // Participants must be created before completeInit() starts the discovery
  // threads; no new participants can be added after that point.
  s_participant = s_domain->createParticipant();
  if (s_participant == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS participant");
    return;
  }

  if (!s_domain->completeInit()) {
    ESP_LOGE(TAG, "Failed to complete RTPS domain init");
    return;
  }

  s_writer = s_domain->createWriter(*s_participant, pub_topic.c_str(),
                                    "std_msgs::msg::String", false);
  if (s_writer == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS writer");
    return;
  }

  s_reader = s_domain->createReader(*s_participant, sub_topic.c_str(),
                                    "std_msgs::msg::String", false);
  if (s_reader == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS reader");
    return;
  }

  if (s_reader->registerCallback(reader_cb, nullptr) == 0) {
    ESP_LOGE(TAG, "Failed to register RTPS reader callback");
    return;
  }

#if CONFIG_RTPS_EXAMPLE_ROLE_INITIATOR
  xTaskCreate(publisher_task, "rtps_pub", 4096, nullptr, 5, nullptr);
#endif

  s_started = true;
  ESP_LOGI(TAG, "started as '%s': pub=%s sub=%s",
           CONFIG_RTPS_EXAMPLE_NODE_NAME, pub_topic.c_str(), sub_topic.c_str());
}


extern "C" void app_main(void) {
  espp::Logger logger({.tag = TAG, .level = espp::Logger::Verbosity::INFO});

  //! [rtps example]
  std::string ip_address;
  espp::WifiSta wifi_sta({
      .ssid = CONFIG_ESP_WIFI_SSID,
      .password = CONFIG_ESP_WIFI_PASSWORD,
      .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
      .on_connected = nullptr,
      .on_disconnected = nullptr,
      .on_got_ip = [&ip_address](ip_event_got_ip_t *eventdata) {
        ip_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
        fmt::print("got IP: {}\n", ip_address);
      },
  });

  logger.info("Waiting for WiFi connection...");
  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  logger.info("WiFi connected, local IP {}", ip_address);

  rtps::Ip4AddressBytes local_ip{0, 0, 0, 0};
  if (std::sscanf(ip_address.c_str(), "%hhu.%hhu.%hhu.%hhu",
                  &local_ip[0], &local_ip[1], &local_ip[2], &local_ip[3]) != 4) {
    logger.error("Failed to parse local IP {}", ip_address);
    return;
  }

  embedded_rtps_start(local_ip);
  //! [rtps example]

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
