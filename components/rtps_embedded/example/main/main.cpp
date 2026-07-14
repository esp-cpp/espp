
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rtps/entities/Domain.h"
#include "logger.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

namespace {

static const char *TAG = "embedded_rtps";
static bool s_started = false;
static rtps::Domain *s_domain = nullptr;
static rtps::Participant *s_participant = nullptr;
static rtps::Writer *s_writer = nullptr;
static rtps::Reader *s_reader = nullptr;
static char s_node_name[16] = "node";

bool is_dhcps_node() {
  return (strcmp(s_node_name, "DHCPS") == 0 ||
          strcmp(s_node_name, "dhcps") == 0);
}

bool is_dhcpc_node() {
  return (strcmp(s_node_name, "DHCPC") == 0 ||
          strcmp(s_node_name, "dhcpc") == 0);
}

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

void reader_cb(void *callee, const rtps::ReaderCacheChange &change) {
  (void)callee;
  char buffer[128] = {0};
  const rtps::DataSize_t copy_len =
      (change.getDataSize() < sizeof(buffer) - 1) ? change.getDataSize()
                                                   : (sizeof(buffer) - 1);

  if (copy_len == 0) {
    return;
  }

  if (!change.copyInto(reinterpret_cast<uint8_t *>(buffer), sizeof(buffer))) {
    return;
  }

  ESP_LOGI(TAG, "RTPS rx (%u B): %s", static_cast<unsigned>(copy_len), buffer);

  if (strcmp(buffer, "who are you? Are you PC?") == 0 && is_dhcpc_node()) {
    if (send_text_message("i am dhcpc")) {
      ESP_LOGI(TAG, "RTPS tx: i am dhcpc");
    } else {
      ESP_LOGW(TAG, "RTPS tx dropped: i am dhcpc");
    }
  } else if (is_dhcps_node()) {
    ESP_LOGI(TAG, "RTPS Rx: %s", buffer);
  }
}

void publisher_task(void *arg) {
  (void)arg;
  while (true) {
    if (is_dhcps_node()) {
      if (!send_text_message("who are you? Are you PC?")) {
        ESP_LOGW(TAG, "RTPS tx dropped (history full or no matched reader)");
      } else {
        ESP_LOGI(TAG, "RTPS tx: who are you? Are you PC?");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

} // namespace

extern "C" void embedded_rtps_start(const char *node_name,
                                      const char *pub_topic,
                                      const char *sub_topic,
                                      const rtps::Ip4AddressBytes &local_ip) {
  if (s_started) {
    return;
  }

  if (node_name == nullptr || pub_topic == nullptr || sub_topic == nullptr) {
    ESP_LOGE(TAG, "RTPS start rejected due to invalid arguments");
    return;
  }

  strncpy(s_node_name, node_name, sizeof(s_node_name) - 1);
  s_node_name[sizeof(s_node_name) - 1] = '\0';

  static rtps::Domain domain(local_ip);
  s_domain = &domain;

  s_participant = s_domain->createParticipant();
  if (s_participant == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS participant");
    return;
  }

  // Complete domain init first so built-in discovery endpoints and worker
  // threads are fully initialized before user endpoints are added.
  if (!s_domain->completeInit()) {
    ESP_LOGE(TAG, "Failed to complete RTPS domain init");
    return;
  }

  s_writer = s_domain->createWriter(*s_participant, pub_topic,
                                    "std_msgs::msg::String", false);
  if (s_writer == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS writer");
    return;
  }

  s_reader = s_domain->createReader(*s_participant, sub_topic,
                                    "std_msgs::msg::String", false);
  if (s_reader == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS reader");
    return;
  }

  if (s_reader->registerCallback(reader_cb, nullptr) == 0) {
    ESP_LOGE(TAG, "Failed to register RTPS reader callback");
    return;
  }

  xTaskCreate(publisher_task, "rtps_pub", 4096, nullptr, 5, nullptr);
  s_started = true;
  ESP_LOGI(TAG, "EmbeddedRTPS started: pub=%s sub=%s", pub_topic, sub_topic);
}


extern "C"  void app_main(void)
{
    espp::Logger logger({.tag = "rtps_example", .level = espp::Logger::Verbosity::INFO});

  //! [rtps example]
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

  logger.info("Waiting for WiFi connection...");
  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  logger.info("WiFi connected, local IP {}", ip_address);

  rtps::Ip4AddressBytes local_ip{0, 0, 0, 0};
  if (std::sscanf(ip_address.c_str(), "%hhu.%hhu.%hhu.%hhu", &local_ip[0],
                  &local_ip[1], &local_ip[2], &local_ip[3]) != 4) {
    logger.error("Failed to parse local IP {}", ip_address);
    return;
  }

  embedded_rtps_start("DHCPS", "rtps_embedded_pub", "rtps_embedded_sub",
                      local_ip);
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
