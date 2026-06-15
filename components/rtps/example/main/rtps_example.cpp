#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include "logger.hpp"
#include "rtps.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

namespace {
constexpr std::string_view kTypeName = "std_msgs/msg/UInt32";

bool run_local_protocol_checks(espp::Logger &logger, const espp::RtpsParticipant &participant) {
  auto announce_message = participant.build_announce_message();
  auto parsed_message = espp::RtpsParticipant::Message::parse(announce_message);
  if (!parsed_message) {
    logger.error("Failed to parse locally built announce message");
    return false;
  }
  logger.info("Built and parsed SPDP announce message with {} submessage(s)",
              parsed_message->submessages.size());

  if (!participant.writers().empty()) {
    auto sedp_publication_message =
        participant.build_sedp_publication_message(participant.writers().front());
    auto parsed_publication_message =
        espp::RtpsParticipant::Message::parse(sedp_publication_message);
    if (!parsed_publication_message) {
      logger.error("Failed to parse locally built SEDP publication message");
      return false;
    }
    logger.info("Built and parsed SEDP publication message with {} submessage(s)",
                parsed_publication_message->submessages.size());
  }

  if (!participant.readers().empty()) {
    auto sedp_subscription_message =
        participant.build_sedp_subscription_message(participant.readers().front());
    auto parsed_subscription_message =
        espp::RtpsParticipant::Message::parse(sedp_subscription_message);
    if (!parsed_subscription_message) {
      logger.error("Failed to parse locally built SEDP subscription message");
      return false;
    }
    logger.info("Built and parsed SEDP subscription message with {} submessage(s)",
                parsed_subscription_message->submessages.size());
  }

  auto uint32_payload = espp::RtpsParticipant::serialize_uint32_cdr(42);
  auto maybe_value = espp::RtpsParticipant::deserialize_uint32_cdr(uint32_payload);
  if (!maybe_value || *maybe_value != 42) {
    logger.error("UInt32 CDR round trip failed");
    return false;
  }
  logger.info("UInt32 CDR round trip succeeded with value {}", *maybe_value);
  return true;
}

bool has_endpoint(std::span<const espp::RtpsParticipant::EndpointProxy> endpoints,
                  std::string_view topic_name, bool is_reader) {
  return std::any_of(endpoints.begin(), endpoints.end(),
                     [topic_name, is_reader](const auto &endpoint) {
                       return endpoint.topic_name == topic_name && endpoint.is_reader == is_reader;
                     });
}
} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "rtps_example", .level = espp::Logger::Verbosity::INFO});

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

  const std::string node_name = CONFIG_RTPS_EXAMPLE_NODE_NAME;
  const std::string topic_prefix = CONFIG_RTPS_EXAMPLE_TOPIC_PREFIX;
  const std::string request_topic = topic_prefix + "/request";
  const std::string response_topic = topic_prefix + "/response";

  std::atomic<uint32_t> request_count{0};
  std::atomic<uint32_t> response_count{0};
  std::atomic<uint32_t> next_request_value{1};
  std::atomic<uint32_t> last_sent_request{0};

  espp::RtpsParticipant participant({
      .node_name = node_name,
      .domain_id = CONFIG_RTPS_EXAMPLE_DOMAIN_ID,
      .participant_id = CONFIG_RTPS_EXAMPLE_PARTICIPANT_ID,
      .advertised_address = ip_address,
      .announce_period = std::chrono::milliseconds(CONFIG_RTPS_EXAMPLE_ANNOUNCE_PERIOD_MS),
      .on_participant_discovered =
          [&logger](const auto &proxy) {
            logger.info("Discovered participant '{}' at {} (meta {}, user {})",
                        proxy.name.empty() ? proxy.guid_prefix.to_string() : proxy.name,
                        proxy.address, proxy.ports.metatraffic_unicast, proxy.ports.user_unicast);
          },
      .on_endpoint_discovered =
          [&logger](const auto &endpoint) {
            logger.info("Discovered remote {} '{}' [{}]", endpoint.is_reader ? "reader" : "writer",
                        endpoint.topic_name, endpoint.type_name);
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

#if CONFIG_RTPS_EXAMPLE_ROLE_INITIATOR
  participant.add_writer({
      .topic_name = request_topic,
      .type_name = std::string(kTypeName),
      .reliability = espp::RtpsParticipant::ReliabilityKind::BEST_EFFORT,
      .entity_index = 0,
  });
  participant.add_reader({
      .topic_name = response_topic,
      .type_name = std::string(kTypeName),
      .reliability = espp::RtpsParticipant::ReliabilityKind::BEST_EFFORT,
      .entity_index = 0,
      .on_uint32_sample =
          [&logger, &response_count, &last_sent_request](uint32_t value) {
            response_count++;
            logger.info("Received response {} (expected {})", value, last_sent_request.load());
          },
  });
#else
  auto *participant_ptr = &participant;
  participant.add_writer({
      .topic_name = response_topic,
      .type_name = std::string(kTypeName),
      .reliability = espp::RtpsParticipant::ReliabilityKind::BEST_EFFORT,
      .entity_index = 0,
  });
  participant.add_reader({
      .topic_name = request_topic,
      .type_name = std::string(kTypeName),
      .reliability = espp::RtpsParticipant::ReliabilityKind::BEST_EFFORT,
      .entity_index = 0,
      .on_uint32_sample =
          [&logger, &request_count, &response_topic, &participant_ptr](uint32_t value) {
            request_count++;
            logger.info("Received request {}, sending response", value);
            if (!participant_ptr->publish_uint32(response_topic, value)) {
              logger.warn("Failed to publish response {}", value);
            }
          },
  });
#endif

  auto ports = participant.ports();
  logger.info("Role: {}",
#if CONFIG_RTPS_EXAMPLE_ROLE_INITIATOR
              "initiator"
#else
              "responder"
#endif
  );
  logger.info("Participant name: {}", node_name);
  logger.info("Participant GUID: {}", participant.participant_guid().to_string());
  logger.info("Domain ID: {}, Participant ID: {}", CONFIG_RTPS_EXAMPLE_DOMAIN_ID,
              CONFIG_RTPS_EXAMPLE_PARTICIPANT_ID);
  logger.info("Topic prefix: {}", topic_prefix);
  logger.info("Request topic: {}, Response topic: {}", request_topic, response_topic);
  logger.info("Ports: meta mc={}, meta uc={}, user mc={}, user uc={}", ports.metatraffic_multicast,
              ports.metatraffic_unicast, ports.user_multicast, ports.user_unicast);

  if (!run_local_protocol_checks(logger, participant)) {
    return;
  }

  if (!participant.start()) {
    logger.error("Failed to start RTPS participant");
    return;
  }

#if CONFIG_RTPS_EXAMPLE_ROLE_INITIATOR
  logger.info("Initiator is waiting for a responder on the same domain/topic prefix...");
  while (true) {
    auto remote_readers = participant.discovered_readers();
    auto remote_writers = participant.discovered_writers();
    bool request_reader_ready = has_endpoint(remote_readers, request_topic, true);
    bool response_writer_ready = has_endpoint(remote_writers, response_topic, false);
    if (!request_reader_ready || !response_writer_ready) {
      logger.info("Waiting for responder endpoints (request_reader={}, response_writer={})",
                  request_reader_ready, response_writer_ready);
      std::this_thread::sleep_for(2s);
      continue;
    }

    auto value = next_request_value.fetch_add(1);
    last_sent_request = value;
    if (participant.publish_uint32(request_topic, value)) {
      logger.info("Published request {} on '{}'", value, request_topic);
    } else {
      logger.warn("Failed to publish request {}", value);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(CONFIG_RTPS_EXAMPLE_PUBLISH_PERIOD_MS));
  }
#else
  logger.info("Responder is ready and will echo '{}' samples back on '{}'", request_topic,
              response_topic);
  while (true) {
    std::this_thread::sleep_for(5s);
    logger.info("Responder status: discovered participants={}, requests handled={}",
                participant.discovered_participants().size(), request_count.load());
  }
#endif
}
