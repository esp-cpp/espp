#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

#include "sdkconfig.h"

#include "nvs_flash.h"

#include "home_assistant.hpp"
#include "logger.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "home_assistant_example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Home Assistant example");

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  std::atomic<bool> got_ip{false};
  espp::WifiSta wifi({
      .ssid = CONFIG_HOME_ASSISTANT_EXAMPLE_WIFI_SSID,
      .password = CONFIG_HOME_ASSISTANT_EXAMPLE_WIFI_PASSWORD,
      .num_connect_retries = 5,
      .on_got_ip =
          [&](ip_event_got_ip_t *event) {
            logger.info("Got IP: {}.{}.{}.{}", IP2STR(&event->ip_info.ip));
            got_ip = true;
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  for (int i = 0; i < 150 && !got_ip; i++) {
    std::this_thread::sleep_for(100ms);
  }

  if (!got_ip) {
    logger.warn("WiFi connection timed out, continuing without network interactions");
  }

  const bool enable_websocket =
#ifdef CONFIG_HOME_ASSISTANT_EXAMPLE_ENABLE_WEBSOCKET
      true;
#else
      false;
#endif

  std::error_code ec;
  espp::HomeAssistant::Config home_assistant_config;
  home_assistant_config.device.name = CONFIG_HOME_ASSISTANT_EXAMPLE_DEVICE_NAME;
  home_assistant_config.device.identifiers = {CONFIG_HOME_ASSISTANT_EXAMPLE_DEVICE_IDENTIFIER};
  home_assistant_config.device.manufacturer = "espp";
  home_assistant_config.device.model = "home_assistant_example";
  home_assistant_config.device.sw_version = "0.1.0";
  home_assistant_config.device.suggested_area = "Lab";
  home_assistant_config.mqtt.broker_uri = CONFIG_HOME_ASSISTANT_EXAMPLE_MQTT_BROKER_URI;
  home_assistant_config.mqtt.username = CONFIG_HOME_ASSISTANT_EXAMPLE_MQTT_USERNAME;
  home_assistant_config.mqtt.password = CONFIG_HOME_ASSISTANT_EXAMPLE_MQTT_PASSWORD;
  home_assistant_config.mqtt.discovery_prefix = CONFIG_HOME_ASSISTANT_EXAMPLE_DISCOVERY_PREFIX;
  home_assistant_config.mqtt.node_id = CONFIG_HOME_ASSISTANT_EXAMPLE_NODE_ID;
  home_assistant_config.mqtt.state_prefix = "espp/home_assistant/example";
  home_assistant_config.api.base_url = CONFIG_HOME_ASSISTANT_EXAMPLE_HOME_ASSISTANT_URL;
  home_assistant_config.api.access_token = CONFIG_HOME_ASSISTANT_EXAMPLE_HOME_ASSISTANT_TOKEN;
  home_assistant_config.api.enable_websocket = enable_websocket;
  home_assistant_config.api.timeout_ms = CONFIG_HOME_ASSISTANT_EXAMPLE_REST_TIMEOUT_MS;
  home_assistant_config.api.on_websocket_message = [&](std::string_view json) {
    logger.info("WS message: {}", std::string(json));
  };
  home_assistant_config.log_level = espp::Logger::Verbosity::INFO;

  espp::HomeAssistant home_assistant(home_assistant_config);

  espp::HomeAssistant::SensorConfig sensor_config;
  sensor_config.object_id = "temperature";
  sensor_config.name = "Temperature";
  sensor_config.unit_of_measurement = "°C";
  sensor_config.device_class = "temperature";
  sensor_config.state_class = "measurement";
  home_assistant.register_sensor(sensor_config, ec);
  if (ec) {
    logger.error("Failed to register sensor: {}", ec.message());
    return;
  }

  bool switch_state = false;
  espp::HomeAssistant::SwitchConfig switch_config;
  switch_config.object_id = "relay";
  switch_config.name = "Relay";
  switch_config.payload_on = "ON";
  switch_config.payload_off = "OFF";
  switch_config.on_command = [&](std::string_view topic, std::string_view payload) {
    logger.info("Switch command on '{}': {}", std::string(topic), std::string(payload));
    switch_state = payload == "ON";
  };
  home_assistant.register_switch(switch_config, ec);
  if (ec) {
    logger.error("Failed to register switch: {}", ec.message());
    return;
  }

  bool has_mqtt = strlen(CONFIG_HOME_ASSISTANT_EXAMPLE_MQTT_BROKER_URI) > 0;
  bool has_api = strlen(CONFIG_HOME_ASSISTANT_EXAMPLE_HOME_ASSISTANT_URL) > 0 &&
                 strlen(CONFIG_HOME_ASSISTANT_EXAMPLE_HOME_ASSISTANT_TOKEN) > 0;

  if (got_ip && (has_mqtt || has_api)) {
    if (!home_assistant.start(ec)) {
      logger.warn("Failed to start Home Assistant integration: {}", ec.message());
    } else {
      logger.info("Home Assistant integration started");
    }
  } else {
    logger.info("Skipping Home Assistant start because WiFi or Home Assistant connectivity is not "
                "configured");
  }

  if (has_mqtt && got_ip) {
    for (int i = 0; i < 100 && !home_assistant.is_mqtt_connected(); i++) {
      std::this_thread::sleep_for(100ms);
    }
    if (!home_assistant.is_mqtt_connected()) {
      logger.warn("MQTT did not connect before publish timeout");
    }
  }

  if (home_assistant.is_mqtt_connected()) {
    for (int i = 0; i < 3; i++) {
      float temperature = 22.5f + static_cast<float>(i) * 0.4f;
      home_assistant.publish_entity_state("temperature", std::to_string(temperature), ec);
      if (ec) {
        logger.warn("Failed to publish temperature: {}", ec.message());
      }

      home_assistant.publish_entity_state("relay", switch_state ? "ON" : "OFF", ec);
      if (ec) {
        logger.warn("Failed to publish relay state: {}", ec.message());
      }

      std::this_thread::sleep_for(1s);
    }
  }

  if (got_ip && has_api) {
    std::string json;
    if (home_assistant.get_config(json, ec)) {
      logger.info("REST /api/config response: {}", json);
    } else {
      logger.warn("REST get_config failed: {}", ec.message());
    }

    if (enable_websocket && home_assistant.is_websocket_connected()) {
      uint32_t subscription_id = 0;
      if (home_assistant.subscribe_websocket_events("state_changed", subscription_id, ec)) {
        logger.info("Subscribed to state_changed events with id {}", subscription_id);
      } else {
        logger.warn("WebSocket subscribe failed: {}", ec.message());
      }
    }
  } else {
    logger.info(
        "Skipping REST / WebSocket demo because Home Assistant URL or token is not configured");
  }

  if (home_assistant.is_mqtt_connected() || home_assistant.is_websocket_connected()) {
    std::this_thread::sleep_for(2s);
  }

  logger.info("Home Assistant example complete");
}
