
#include <cstdio>
#include <cstring>
#include <thread>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp32-ethernet-kit.hpp"
#include "logger.hpp"
#include "rtps/entities/Domain.h"
#include "ucdr/microcdr.h"

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

  // CDR_LE encapsulation header (scheme 0x00 0x01 + 2 zero option bytes) followed by the string.
  uint8_t cdr_buf[4 + 4 + 128]; // encap + CDR length prefix + max text (127 chars + null)
  cdr_buf[0] = 0x00;
  cdr_buf[1] = 0x01;
  cdr_buf[2] = 0x00;
  cdr_buf[3] = 0x00;

  ucdrBuffer ub;
  ucdr_init_buffer_origin_offset_endian(&ub, cdr_buf, sizeof(cdr_buf), 0, 4,
                                        UCDR_LITTLE_ENDIANNESS);
  if (!ucdr_serialize_string(&ub, text) || ucdr_buffer_has_error(&ub)) {
    return false;
  }

  const auto total = static_cast<rtps::DataSize_t>(4 + ucdr_buffer_length(&ub));
  const rtps::CacheChange *change = s_writer->newChange(rtps::ChangeKind_t::ALIVE, cdr_buf, total);
  return change != nullptr;
}

void reader_cb(void * /*callee*/, const rtps::ReaderCacheChange &change) {
  const rtps::DataSize_t size = change.getDataSize();
  if (size < 5) { // need at least the 4-byte encapsulation header plus data
    return;
  }

  uint8_t raw[4 + 4 + 128]; // encap + CDR length prefix + max text (127 chars + null)
  const auto copy_size = static_cast<rtps::DataSize_t>(size <= sizeof(raw) ? size : sizeof(raw));
  if (!change.copyInto(raw, copy_size)) {
    ESP_LOGI(TAG, "Failed to copy RTPS change data into buffer");
    return;
  }

  // Decode the CDR string, skipping the 4-byte CDR encapsulation header.
  ucdrBuffer ub;
  ucdr_init_buffer_origin_offset_endian(&ub, raw, copy_size, 0, 4, UCDR_LITTLE_ENDIANNESS);
  char text[128] = {0};
  if (!ucdr_deserialize_string(&ub, text, sizeof(text)) || ucdr_buffer_has_error(&ub)) {
    ESP_LOGI(TAG, "Failed to deserialize RTPS change data");
    return;
  }

  ESP_LOGI(TAG, "rx: %s", text);
}

void publisher_task(void * /*arg*/) {
  uint32_t counter = 0;
  while (true) {
    char msg[32];
    snprintf(msg, sizeof(msg), "msg %u", static_cast<unsigned>(counter++));
    if (!send_text_message(msg)) {
      ESP_LOGW(TAG, "tx dropped (history full or no matched reader)");
    } else {
      ESP_LOGI(TAG, "tx: %s", msg);
    }
    vTaskDelay(pdMS_TO_TICKS(CONFIG_RTPS_EXAMPLE_ANNOUNCE_PERIOD_MS));
  }
}

} // namespace

extern "C" void embedded_rtps_start(const rtps::Ip4AddressBytes &local_ip) {
  if (s_started) {
    return;
  }

  constexpr const char *pub_topic = "mcu_to_pc";
  constexpr const char *sub_topic = "pc_to_mcu";

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

  s_writer = s_domain->createWriter(*s_participant, pub_topic, "std_msgs::msg::String", true);
  if (s_writer == nullptr) {
    ESP_LOGE(TAG, "Failed to create RTPS writer");
    return;
  }

  s_reader = s_domain->createReader(*s_participant, sub_topic, "std_msgs::msg::String", false);
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
  ESP_LOGI(TAG, "started: pub=%s sub=%s", pub_topic, sub_topic);
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
