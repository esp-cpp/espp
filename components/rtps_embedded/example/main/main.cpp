
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp32-ethernet-kit.hpp"
#include "logger.hpp"
#include "rtps/entities/Domain.h"

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
  const rtps::CacheChange *change =
      s_writer->newChange(rtps::ChangeKind_t::ALIVE, reinterpret_cast<const uint8_t *>(text),
                          static_cast<rtps::DataSize_t>(len + 1));

  return change != nullptr;
}

void reader_cb(void * /*callee*/, const rtps::ReaderCacheChange &change) {
  char buffer[128] = {0};
  const rtps::DataSize_t copy_len =
      (change.getDataSize() < static_cast<rtps::DataSize_t>(sizeof(buffer) - 1))
          ? change.getDataSize()
          : static_cast<rtps::DataSize_t>(sizeof(buffer) - 1);

  if (copy_len == 0 || !change.copyInto(reinterpret_cast<uint8_t *>(buffer), sizeof(buffer))) {
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

  s_writer =
      s_domain->createWriter(*s_participant, pub_topic.c_str(), "std_msgs::msg::String", false);
  if (s_writer == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS writer");
    return;
  }

  s_reader =
      s_domain->createReader(*s_participant, sub_topic.c_str(), "std_msgs::msg::String", false);
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
  ESP_LOGI(TAG, "started as '%s': pub=%s sub=%s", CONFIG_RTPS_EXAMPLE_NODE_NAME, pub_topic.c_str(),
           sub_topic.c_str());
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = TAG, .level = espp::Logger::Verbosity::INFO});

  //! [rtps example]
  auto &board = espp::Esp32EthernetKit::get();

  // Static IP 192.168.4.1/24 — ip_info zero-initialised uses that default.
  espp::Esp32EthernetKit::ServerConfig srv_cfg;

  bool eth_ok = board.initialize_ethernet({
      .mode = espp::Esp32EthernetKit::DhcpMode::SERVER,
      .server_config = srv_cfg,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            logger.info("Ethernet DHCP server ready at {}.{}.{}.{}", esp_ip4_addr1_16(&ip),
                        esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
          },
      .on_lost_ip = [&]() { logger.warn("Ethernet lost IP"); },
  });

  if (!eth_ok) {
    logger.error("Ethernet initialization failed");
    return;
  }

  logger.info("Waiting for Ethernet link...");
  while (!board.is_ethernet_connected()) {
    std::this_thread::sleep_for(100ms);
  }

  auto eth_ip = board.ethernet_ip();
  logger.info("Ethernet up, IP {}.{}.{}.{}", esp_ip4_addr1_16(&eth_ip), esp_ip4_addr2_16(&eth_ip),
              esp_ip4_addr3_16(&eth_ip), esp_ip4_addr4_16(&eth_ip));

  rtps::Ip4AddressBytes local_ip{
      static_cast<uint8_t>(esp_ip4_addr1_16(&eth_ip)),
      static_cast<uint8_t>(esp_ip4_addr2_16(&eth_ip)),
      static_cast<uint8_t>(esp_ip4_addr3_16(&eth_ip)),
      static_cast<uint8_t>(esp_ip4_addr4_16(&eth_ip)),
  };

  embedded_rtps_start(local_ip);
  //! [rtps example]

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
