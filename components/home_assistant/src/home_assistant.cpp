#include "home_assistant.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"

using namespace espp;

namespace {

bool starts_with(std::string_view value, std::string_view prefix) {
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

std::string trim_slashes(std::string_view value) {
  size_t start = 0;
  while (start < value.size() && value[start] == '/') {
    start++;
  }
  size_t end = value.size();
  while (end > start && value[end - 1] == '/') {
    end--;
  }
  return std::string(value.substr(start, end - start));
}

std::string trim_trailing_slash(std::string_view value) {
  size_t end = value.size();
  while (end > 0 && value[end - 1] == '/') {
    end--;
  }
  return std::string(value.substr(0, end));
}

std::string sanitize_token(std::string_view value) {
  std::string out;
  out.reserve(value.size());
  bool last_was_separator = false;
  for (char c : value) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
      last_was_separator = false;
    } else if (!last_was_separator) {
      out.push_back('_');
      last_was_separator = true;
    }
  }
  while (!out.empty() && out.front() == '_') {
    out.erase(out.begin());
  }
  while (!out.empty() && out.back() == '_') {
    out.pop_back();
  }
  return out.empty() ? "entity" : out;
}

std::string normalize_topic(std::string_view prefix, std::string_view object_id,
                            std::string_view suffix) {
  std::string topic = trim_trailing_slash(prefix);
  topic += "/";
  topic += sanitize_token(object_id);
  auto normalized_suffix = trim_slashes(suffix);
  if (!normalized_suffix.empty()) {
    topic += "/";
    topic += normalized_suffix;
  }
  return topic;
}

std::string join_path(std::string_view base, std::string_view path) {
  std::string url = trim_trailing_slash(base);
  if (path.empty() || path.front() != '/') {
    url += "/";
  }
  url += path;
  return url;
}

void add_string_if_not_empty(cJSON *object, const char *name, const std::string &value) {
  if (!value.empty()) {
    cJSON_AddStringToObject(object, name, value.c_str());
  }
}

std::string json_to_string(cJSON *object) {
  char *buffer = cJSON_PrintUnformatted(object);
  if (!buffer) {
    return {};
  }
  std::string json(buffer);
  cJSON_free(buffer);
  return json;
}

std::string make_entity_key(std::string_view component, std::string_view object_id) {
  return std::string(component) + ":" + sanitize_token(object_id);
}

std::string to_websocket_url(std::string_view base_url, std::string_view explicit_url) {
  if (!explicit_url.empty()) {
    return trim_trailing_slash(explicit_url);
  }
  auto normalized = trim_trailing_slash(base_url);
  if (starts_with(normalized, "http://")) {
    normalized.replace(0, 4, "ws");
  } else if (starts_with(normalized, "https://")) {
    normalized.replace(0, 5, "wss");
  }
  if (!starts_with(normalized, "ws://") && !starts_with(normalized, "wss://")) {
    return {};
  }
  if (!starts_with(normalized, "ws://") && !starts_with(normalized, "wss://")) {
    return {};
  }
  if (normalized.find("/api/websocket") == std::string::npos) {
    normalized += "/api/websocket";
  }
  return normalized;
}

} // namespace

HomeAssistant::HomeAssistant(const Config &config)
    : BaseComponent("HomeAssistant", config.log_level)
    , config_(config) {}

HomeAssistant::~HomeAssistant() { stop(); }

void HomeAssistant::set_config(const Config &config) {
  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = config;
  set_log_level(config.log_level);
}

HomeAssistant::Config HomeAssistant::get_config() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return config_;
}

HomeAssistant::Config HomeAssistant::get_config_locked() const { return config_; }

bool HomeAssistant::start(std::error_code &ec) {
  if (started_) {
    ec = HomeAssistantErrc::already_started;
    return false;
  }

  auto config = get_config();
  started_ = true;

  if (!config.mqtt.broker_uri.empty() && !start_mqtt(ec)) {
    started_ = false;
    return false;
  }

  if (config.api.enable_websocket) {
    if (!connect_websocket(ec)) {
      stop();
      return false;
    }
  }

  ec.clear();
  return true;
}

void HomeAssistant::stop() {
  if (!started_) {
    return;
  }

  disconnect_websocket();
  stop_mqtt();

  started_ = false;
}

bool HomeAssistant::start_mqtt(std::error_code &ec) {
  auto config = get_config();
  if (config.mqtt.broker_uri.empty()) {
    ec = HomeAssistantErrc::invalid_configuration;
    return false;
  }

  esp_mqtt_client_config_t mqtt_config = {};
  mqtt_config.broker.address.uri = config.mqtt.broker_uri.c_str();
  mqtt_config.credentials.username =
      config.mqtt.username.empty() ? nullptr : config.mqtt.username.c_str();
  mqtt_config.credentials.client_id =
      config.mqtt.client_id.empty() ? nullptr : config.mqtt.client_id.c_str();
  mqtt_config.credentials.authentication.password =
      config.mqtt.password.empty() ? nullptr : config.mqtt.password.c_str();
  mqtt_config.session.keepalive = config.mqtt.keepalive_seconds;
  mqtt_config.network.timeout_ms = config.mqtt.network_timeout_ms;
  mqtt_config.network.disable_auto_reconnect = !config.mqtt.auto_reconnect;
  mqtt_config.buffer.size = config.mqtt.buffer_size;
  mqtt_config.buffer.out_size = config.mqtt.buffer_size;
  mqtt_config.task.stack_size = config.mqtt.task_stack_size;
  if (config.mqtt.publish_availability) {
    auto availability_topic =
        config.mqtt.availability_topic.empty()
            ? normalize_topic(config.mqtt.state_prefix, "device", "availability")
            : config.mqtt.availability_topic;
    mqtt_config.session.last_will.topic = availability_topic.c_str();
    mqtt_config.session.last_will.msg = config.availability_payload_offline.c_str();
    mqtt_config.session.last_will.msg_len = config.availability_payload_offline.size();
    mqtt_config.session.last_will.qos = config.mqtt.qos;
    mqtt_config.session.last_will.retain = 1;
  }
  if (config.mqtt.use_crt_bundle && (starts_with(config.mqtt.broker_uri, "mqtts://") ||
                                     starts_with(config.mqtt.broker_uri, "wss://"))) {
    mqtt_config.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
  }

  mqtt_client_ = esp_mqtt_client_init(&mqtt_config);
  if (!mqtt_client_) {
    ec = HomeAssistantErrc::mqtt_error;
    return false;
  }

  auto err = esp_mqtt_client_register_event(mqtt_client_, MQTT_EVENT_ANY, mqtt_event_handler, this);
  if (err != ESP_OK) {
    logger_.error("Failed to register MQTT events: {}", esp_err_to_name(err));
    esp_mqtt_client_destroy(mqtt_client_);
    mqtt_client_ = nullptr;
    ec = HomeAssistantErrc::mqtt_error;
    return false;
  }

  err = esp_mqtt_client_start(mqtt_client_);
  if (err != ESP_OK) {
    logger_.error("Failed to start MQTT client: {}", esp_err_to_name(err));
    esp_mqtt_client_destroy(mqtt_client_);
    mqtt_client_ = nullptr;
    ec = HomeAssistantErrc::mqtt_error;
    return false;
  }

  ec.clear();
  return true;
}

void HomeAssistant::stop_mqtt() {
  if (!mqtt_client_) {
    mqtt_connected_ = false;
    return;
  }

  if (mqtt_connected_) {
    std::error_code ec;
    publish_availability(get_config().availability_payload_offline, ec);
  }

  esp_mqtt_client_stop(mqtt_client_);
  esp_mqtt_client_destroy(mqtt_client_);
  mqtt_client_ = nullptr;
  mqtt_connected_ = false;
}

bool HomeAssistant::connect_websocket(std::error_code &ec) {
  auto config = get_config();
  auto websocket_url = to_websocket_url(config.api.base_url, config.api.websocket_url);
  if (websocket_url.empty() || config.api.access_token.empty()) {
    ec = HomeAssistantErrc::invalid_configuration;
    return false;
  }

  disconnect_websocket();

  websocket_last_error_.clear();
  websocket_last_error_message_.clear();
  websocket_authenticated_ = false;
  websocket_transport_connected_ = false;

  esp_websocket_client_config_t websocket_config = {};
  websocket_config.uri = websocket_url.c_str();
  websocket_config.user_context = this;
  websocket_config.disable_auto_reconnect = true;
  websocket_config.task_stack = config.api.websocket_task_stack_size;
  websocket_config.buffer_size = config.api.websocket_buffer_size;
  websocket_config.network_timeout_ms = config.api.timeout_ms;
  websocket_config.ping_interval_sec = config.api.websocket_ping_interval_sec;
  websocket_config.skip_cert_common_name_check = config.api.skip_cert_common_name_check;
  if (config.api.use_crt_bundle && starts_with(websocket_url, "wss://")) {
    websocket_config.crt_bundle_attach = esp_crt_bundle_attach;
  }

  websocket_client_ = esp_websocket_client_init(&websocket_config);
  if (!websocket_client_) {
    ec = HomeAssistantErrc::websocket_error;
    return false;
  }

  auto err = esp_websocket_register_events(websocket_client_, WEBSOCKET_EVENT_ANY,
                                           websocket_event_handler, this);
  if (err != ESP_OK) {
    logger_.error("Failed to register WebSocket events: {}", esp_err_to_name(err));
    esp_websocket_client_destroy(websocket_client_);
    websocket_client_ = nullptr;
    ec = HomeAssistantErrc::websocket_error;
    return false;
  }

  err = esp_websocket_client_start(websocket_client_);
  if (err != ESP_OK) {
    logger_.error("Failed to start WebSocket client: {}", esp_err_to_name(err));
    esp_websocket_client_destroy(websocket_client_);
    websocket_client_ = nullptr;
    ec = HomeAssistantErrc::websocket_error;
    return false;
  }

  auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(config.api.timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (websocket_authenticated_) {
      ec.clear();
      return true;
    }
    if (websocket_last_error_) {
      ec = websocket_last_error_;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  websocket_last_error_ = HomeAssistantErrc::timeout;
  ec = websocket_last_error_;
  return false;
}

void HomeAssistant::disconnect_websocket() {
  if (!websocket_client_) {
    websocket_authenticated_ = false;
    websocket_transport_connected_ = false;
    return;
  }

  esp_websocket_client_stop(websocket_client_);
  esp_websocket_client_destroy(websocket_client_);
  websocket_client_ = nullptr;
  websocket_authenticated_ = false;
  websocket_transport_connected_ = false;
}

bool HomeAssistant::send_websocket_text(std::string_view json, std::error_code &ec,
                                        bool require_auth) {
  if (!websocket_client_ || !websocket_transport_connected_) {
    ec = HomeAssistantErrc::websocket_error;
    return false;
  }
  if (require_auth && !websocket_authenticated_) {
    ec = HomeAssistantErrc::websocket_auth_failed;
    return false;
  }
  int written = esp_websocket_client_send_text(websocket_client_, json.data(), json.size(),
                                               pdMS_TO_TICKS(1000));
  if (written < 0) {
    ec = HomeAssistantErrc::websocket_error;
    return false;
  }
  ec.clear();
  return true;
}

bool HomeAssistant::send_websocket_command(std::string_view json, std::error_code &ec) {
  return send_websocket_text(json, ec, true);
}

bool HomeAssistant::subscribe_websocket_events(std::string_view event_type,
                                               uint32_t &subscription_id, std::error_code &ec) {
  cJSON *request = cJSON_CreateObject();
  if (!request) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  subscription_id = websocket_next_id_.fetch_add(1);
  cJSON_AddNumberToObject(request, "id", static_cast<double>(subscription_id));
  cJSON_AddStringToObject(request, "type", "subscribe_events");
  if (!event_type.empty()) {
    cJSON_AddStringToObject(request, "event_type", std::string(event_type).c_str());
  }

  auto json = json_to_string(request);
  cJSON_Delete(request);
  if (json.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return send_websocket_command(json, ec);
}

bool HomeAssistant::perform_http_request(esp_http_client_method_t method, std::string_view path,
                                         std::string_view body, std::string &response,
                                         std::error_code &ec) {
  auto config = get_config();
  if (config.api.base_url.empty() || config.api.access_token.empty()) {
    ec = HomeAssistantErrc::invalid_configuration;
    return false;
  }

  auto url = join_path(config.api.base_url, path);
  response.clear();

  esp_http_client_config_t http_config = {};
  http_config.url = url.c_str();
  http_config.method = method;
  http_config.timeout_ms = config.api.timeout_ms;
  http_config.skip_cert_common_name_check = config.api.skip_cert_common_name_check;
  if (config.api.use_crt_bundle && starts_with(url, "https://")) {
    http_config.crt_bundle_attach = esp_crt_bundle_attach;
  }

  auto client = esp_http_client_init(&http_config);
  if (!client) {
    ec = HomeAssistantErrc::http_error;
    return false;
  }

  auto bearer = std::string("Bearer ") + config.api.access_token;
  esp_http_client_set_header(client, "Authorization", bearer.c_str());
  esp_http_client_set_header(client, "Accept", "application/json");
  if (!body.empty()) {
    esp_http_client_set_header(client, "Content-Type", "application/json");
  }

  auto err = esp_http_client_open(client, body.size());
  if (err != ESP_OK) {
    logger_.error("Failed to open HTTP request: {}", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    ec = HomeAssistantErrc::http_error;
    return false;
  }

  if (!body.empty()) {
    auto written = esp_http_client_write(client, body.data(), body.size());
    if (written < 0) {
      logger_.error("Failed to write HTTP request body");
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      ec = HomeAssistantErrc::http_error;
      return false;
    }
  }

  esp_http_client_fetch_headers(client);

  char buffer[256];
  while (true) {
    int read = esp_http_client_read(client, buffer, sizeof(buffer));
    if (read < 0) {
      logger_.error("Failed to read HTTP response");
      esp_http_client_close(client);
      esp_http_client_cleanup(client);
      ec = HomeAssistantErrc::http_error;
      return false;
    }
    if (read == 0) {
      break;
    }
    response.append(buffer, read);
  }

  int status_code = esp_http_client_get_status_code(client);
  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if (status_code < 200 || status_code >= 300) {
    logger_.warn("Home Assistant HTTP request returned {}", status_code);
    ec = HomeAssistantErrc::http_error;
    return false;
  }

  ec.clear();
  return true;
}

bool HomeAssistant::get_config(std::string &json, std::error_code &ec) {
  return perform_http_request(HTTP_METHOD_GET, "/api/config", "", json, ec);
}

bool HomeAssistant::get_state(std::string_view entity_id, std::string &json, std::error_code &ec) {
  if (entity_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }
  auto path = std::string("/api/states/") + std::string(entity_id);
  return perform_http_request(HTTP_METHOD_GET, path, "", json, ec);
}

bool HomeAssistant::set_state(std::string_view entity_id, std::string_view state,
                              std::string_view attributes_json, std::string &json,
                              std::error_code &ec) {
  if (entity_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  cJSON *request = cJSON_CreateObject();
  if (!request) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  cJSON_AddStringToObject(request, "state", std::string(state).c_str());

  if (!attributes_json.empty()) {
    cJSON *attributes = cJSON_ParseWithLength(attributes_json.data(), attributes_json.size());
    if (!attributes || !cJSON_IsObject(attributes)) {
      if (attributes) {
        cJSON_Delete(attributes);
      }
      cJSON_Delete(request);
      ec = HomeAssistantErrc::json_error;
      return false;
    }
    cJSON_AddItemToObject(request, "attributes", attributes);
  }

  auto body = json_to_string(request);
  cJSON_Delete(request);
  if (body.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  auto path = std::string("/api/states/") + std::string(entity_id);
  return perform_http_request(HTTP_METHOD_POST, path, body, json, ec);
}

bool HomeAssistant::call_service(std::string_view domain, std::string_view service,
                                 std::string_view service_data_json, std::string &json,
                                 std::error_code &ec) {
  if (domain.empty() || service.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  std::string body;
  if (service_data_json.empty()) {
    body = "{}";
  } else {
    cJSON *service_data = cJSON_ParseWithLength(service_data_json.data(), service_data_json.size());
    if (!service_data || !cJSON_IsObject(service_data)) {
      if (service_data) {
        cJSON_Delete(service_data);
      }
      ec = HomeAssistantErrc::json_error;
      return false;
    }
    body = json_to_string(service_data);
    cJSON_Delete(service_data);
    if (body.empty()) {
      ec = HomeAssistantErrc::json_error;
      return false;
    }
  }

  auto path = std::string("/api/services/") + std::string(domain) + "/" + std::string(service);
  return perform_http_request(HTTP_METHOD_POST, path, body, json, ec);
}

std::string HomeAssistant::make_state_topic(std::string_view object_id,
                                            std::string_view suffix) const {
  auto config = get_config();
  return normalize_topic(config.mqtt.state_prefix, object_id, suffix);
}

std::string HomeAssistant::make_command_topic(std::string_view object_id,
                                              std::string_view suffix) const {
  auto config = get_config();
  return normalize_topic(config.mqtt.state_prefix, object_id, suffix);
}

std::string HomeAssistant::make_discovery_topic(std::string_view component,
                                                std::string_view object_id) const {
  auto config = get_config();
  auto sanitized_object_id = sanitize_token(object_id);
  auto discovery_prefix = trim_trailing_slash(config.mqtt.discovery_prefix);
  if (config.mqtt.node_id.empty()) {
    return discovery_prefix + "/" + std::string(component) + "/" + sanitized_object_id + "/config";
  }
  return discovery_prefix + "/" + std::string(component) + "/" +
         sanitize_token(config.mqtt.node_id) + "/" + sanitized_object_id + "/config";
}

bool HomeAssistant::publish_state(std::string_view topic, std::string_view state,
                                  std::error_code &ec, bool retain) {
  if (!mqtt_client_ || !mqtt_connected_) {
    ec = HomeAssistantErrc::mqtt_not_connected;
    return false;
  }

  auto config = get_config();
  int message_id = esp_mqtt_client_publish(mqtt_client_, std::string(topic).c_str(), state.data(),
                                           state.size(), config.mqtt.qos, retain ? 1 : 0);
  if (message_id < 0) {
    ec = HomeAssistantErrc::mqtt_error;
    return false;
  }
  ec.clear();
  return true;
}

bool HomeAssistant::publish_entity_state(std::string_view object_id, std::string_view state,
                                         std::error_code &ec, bool retain) {
  std::lock_guard<std::mutex> lock(entity_mutex_);
  auto object_it = object_lookup_.find(sanitize_token(object_id));
  if (object_it == object_lookup_.end()) {
    ec = HomeAssistantErrc::entity_not_found;
    return false;
  }
  auto entity_it = entities_.find(object_it->second);
  if (entity_it == entities_.end() || entity_it->second.primary_state_topic.empty()) {
    ec = HomeAssistantErrc::entity_not_found;
    return false;
  }
  return publish_state(entity_it->second.primary_state_topic, state, ec, retain);
}

bool HomeAssistant::publish_availability(std::string_view payload, std::error_code &ec) {
  auto config = get_config();
  if (!config.mqtt.publish_availability) {
    ec.clear();
    return true;
  }
  auto topic = config.mqtt.availability_topic.empty()
                   ? normalize_topic(config.mqtt.state_prefix, "device", "availability")
                   : config.mqtt.availability_topic;
  return publish_state(topic, payload, ec, true);
}

bool HomeAssistant::publish_discovery(const RegisteredEntity &entity, std::error_code &ec) {
  if (!mqtt_client_ || !mqtt_connected_) {
    ec = HomeAssistantErrc::mqtt_not_connected;
    return false;
  }
  auto config = get_config();
  int message_id = esp_mqtt_client_publish(mqtt_client_, entity.discovery_topic.c_str(),
                                           entity.discovery_payload.c_str(),
                                           entity.discovery_payload.size(), config.mqtt.qos, 1);
  if (message_id < 0) {
    ec = HomeAssistantErrc::mqtt_error;
    return false;
  }
  for (const auto &[topic, _] : entity.command_callbacks) {
    esp_mqtt_client_subscribe(mqtt_client_, topic.c_str(), config.mqtt.qos);
  }
  ec.clear();
  return true;
}

bool HomeAssistant::register_entity(RegisteredEntity entity, std::error_code &ec) {
  std::lock_guard<std::mutex> lock(entity_mutex_);
  if (entities_.contains(entity.key) || object_lookup_.contains(sanitize_token(entity.object_id))) {
    ec = HomeAssistantErrc::entity_already_exists;
    return false;
  }

  object_lookup_[sanitize_token(entity.object_id)] = entity.key;
  entities_.emplace(entity.key, std::move(entity));
  auto stored = entities_.find(object_lookup_[sanitize_token(entity.object_id)]);
  if (mqtt_connected_ && stored != entities_.end()) {
    return publish_discovery(stored->second, ec);
  }

  ec.clear();
  return true;
}

bool HomeAssistant::remove_entity(std::string_view component, std::string_view object_id,
                                  std::error_code &ec) {
  std::lock_guard<std::mutex> lock(entity_mutex_);
  auto key = make_entity_key(component, object_id);
  auto it = entities_.find(key);
  if (it == entities_.end()) {
    ec = HomeAssistantErrc::entity_not_found;
    return false;
  }

  if (mqtt_client_ && mqtt_connected_) {
    auto config = get_config();
    int message_id = esp_mqtt_client_publish(mqtt_client_, it->second.discovery_topic.c_str(), "",
                                             0, config.mqtt.qos, 1);
    if (message_id < 0) {
      ec = HomeAssistantErrc::mqtt_error;
      return false;
    }
  }

  object_lookup_.erase(sanitize_token(object_id));
  entities_.erase(it);
  ec.clear();
  return true;
}

void HomeAssistant::handle_mqtt_command(std::string_view topic, std::string_view payload) {
  CommandCallback callback{nullptr};
  {
    std::lock_guard<std::mutex> lock(entity_mutex_);
    for (auto &[_, entity] : entities_) {
      auto callback_it = entity.command_callbacks.find(std::string(topic));
      if (callback_it != entity.command_callbacks.end() && callback_it->second) {
        callback = callback_it->second;
        break;
      }
    }
  }
  if (callback) {
    callback(topic, payload);
  }
}

void HomeAssistant::handle_websocket_message(std::string_view json) {
  cJSON *root = cJSON_ParseWithLength(json.data(), json.size());
  if (!root) {
    logger_.warn("Failed to parse WebSocket message");
    return;
  }

  auto type = cJSON_GetObjectItemCaseSensitive(root, "type");
  if (cJSON_IsString(type) && type->valuestring) {
    std::string type_string = type->valuestring;
    if (type_string == "auth_required") {
      auto config = get_config();
      cJSON *auth = cJSON_CreateObject();
      if (auth) {
        cJSON_AddStringToObject(auth, "type", "auth");
        cJSON_AddStringToObject(auth, "access_token", config.api.access_token.c_str());
        auto auth_json = json_to_string(auth);
        cJSON_Delete(auth);
        if (!auth_json.empty()) {
          std::error_code ec;
          send_websocket_text(auth_json, ec, false);
        }
      }
    } else if (type_string == "auth_ok") {
      websocket_authenticated_ = true;
      websocket_last_error_.clear();
      logger_.info("Home Assistant WebSocket authenticated");
    } else if (type_string == "auth_invalid") {
      websocket_last_error_ = HomeAssistantErrc::websocket_auth_failed;
      websocket_authenticated_ = false;
      auto message = cJSON_GetObjectItemCaseSensitive(root, "message");
      if (cJSON_IsString(message) && message->valuestring) {
        websocket_last_error_message_ = message->valuestring;
      }
      logger_.error("Home Assistant WebSocket auth failed: {}", websocket_last_error_message_);
    }
  }

  auto config = get_config();
  if (config.api.on_websocket_message) {
    config.api.on_websocket_message(json);
  }
  cJSON_Delete(root);
}

void HomeAssistant::mqtt_event_handler(void *handler_args, esp_event_base_t, int32_t event_id,
                                       void *event_data) {
  auto *self = static_cast<HomeAssistant *>(handler_args);
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  if (!self || !event) {
    return;
  }

  switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
  case MQTT_EVENT_CONNECTED: {
    self->mqtt_connected_ = true;
    self->logger_.info("Connected to MQTT broker");
    std::error_code ec;
    self->publish_availability(self->get_config().availability_payload_online, ec);
    std::lock_guard<std::mutex> lock(self->entity_mutex_);
    for (const auto &[_, entity] : self->entities_) {
      self->publish_discovery(entity, ec);
    }
    break;
  }
  case MQTT_EVENT_DISCONNECTED:
    self->mqtt_connected_ = false;
    self->logger_.warn("Disconnected from MQTT broker");
    break;
  case MQTT_EVENT_DATA: {
    std::string topic(event->topic, event->topic_len);
    std::string payload(event->data, event->data_len);
    self->handle_mqtt_command(topic, payload);
    break;
  }
  case MQTT_EVENT_ERROR:
    self->logger_.warn("MQTT client reported an error");
    break;
  default:
    break;
  }
}

void HomeAssistant::websocket_event_handler(void *handler_args, esp_event_base_t, int32_t event_id,
                                            void *event_data) {
  auto *self = static_cast<HomeAssistant *>(handler_args);
  auto *event = static_cast<esp_websocket_event_data_t *>(event_data);
  if (!self || !event) {
    return;
  }

  switch (static_cast<esp_websocket_event_id_t>(event_id)) {
  case WEBSOCKET_EVENT_CONNECTED:
    self->websocket_transport_connected_ = true;
    self->logger_.info("Connected to Home Assistant WebSocket endpoint");
    break;
  case WEBSOCKET_EVENT_DISCONNECTED:
  case WEBSOCKET_EVENT_CLOSED:
    self->websocket_transport_connected_ = false;
    self->websocket_authenticated_ = false;
    self->logger_.warn("Home Assistant WebSocket disconnected");
    break;
  case WEBSOCKET_EVENT_ERROR:
    self->websocket_last_error_ = HomeAssistantErrc::websocket_error;
    self->logger_.warn("Home Assistant WebSocket error");
    break;
  case WEBSOCKET_EVENT_DATA: {
    std::lock_guard<std::mutex> lock(self->websocket_mutex_);
    if (event->payload_offset == 0) {
      self->websocket_buffer_.clear();
    }
    self->websocket_buffer_.append(event->data_ptr, event->data_len);
    if ((event->payload_offset + event->data_len) >= event->payload_len || event->fin) {
      auto complete_message = self->websocket_buffer_;
      self->websocket_buffer_.clear();
      self->handle_websocket_message(complete_message);
    }
    break;
  }
  default:
    break;
  }
}

static cJSON *build_common_discovery_payload(const HomeAssistant::Config &config,
                                             const HomeAssistant::EntityConfig &entity,
                                             std::string_view component, std::string_view object_id,
                                             std::string_view unique_id) {
  cJSON *root = cJSON_CreateObject();
  if (!root) {
    return nullptr;
  }

  cJSON_AddStringToObject(root, "object_id", std::string(object_id).c_str());
  cJSON_AddStringToObject(root, "unique_id", std::string(unique_id).c_str());
  cJSON_AddStringToObject(root, "name",
                          (entity.name.empty() ? std::string(object_id) : entity.name).c_str());
  cJSON_AddBoolToObject(root, "enabled_by_default", entity.enabled_by_default);
  add_string_if_not_empty(root, "icon", entity.icon);
  add_string_if_not_empty(root, "entity_category", entity.entity_category);
  add_string_if_not_empty(root, "json_attributes_topic", entity.json_attributes_topic);

  auto availability_topic =
      entity.availability_topic.empty()
          ? (config.mqtt.availability_topic.empty()
                 ? normalize_topic(config.mqtt.state_prefix, "device", "availability")
                 : config.mqtt.availability_topic)
          : entity.availability_topic;
  if (!entity.availability_topic.empty() || config.mqtt.publish_availability) {
    add_string_if_not_empty(root, "availability_topic", availability_topic);
    add_string_if_not_empty(root, "payload_available", config.availability_payload_online);
    add_string_if_not_empty(root, "payload_not_available", config.availability_payload_offline);
  }

  cJSON *device = cJSON_CreateObject();
  if (!device) {
    cJSON_Delete(root);
    return nullptr;
  }
  add_string_if_not_empty(device, "name", config.device.name);
  add_string_if_not_empty(device, "manufacturer", config.device.manufacturer);
  add_string_if_not_empty(device, "model", config.device.model);
  add_string_if_not_empty(device, "sw_version", config.device.sw_version);
  add_string_if_not_empty(device, "hw_version", config.device.hw_version);
  add_string_if_not_empty(device, "suggested_area", config.device.suggested_area);
  add_string_if_not_empty(device, "configuration_url", config.device.configuration_url);

  cJSON *identifiers = cJSON_CreateArray();
  for (const auto &identifier : config.device.identifiers) {
    cJSON_AddItemToArray(identifiers, cJSON_CreateString(identifier.c_str()));
  }
  cJSON_AddItemToObject(device, "identifiers", identifiers);
  cJSON_AddItemToObject(root, "device", device);

  (void)component;
  return root;
}

static std::string default_unique_id(const HomeAssistant::Config &config,
                                     std::string_view object_id) {
  if (!config.device.identifiers.empty()) {
    return sanitize_token(config.device.identifiers.front()) + "_" + sanitize_token(object_id);
  }
  if (!config.device.name.empty()) {
    return sanitize_token(config.device.name) + "_" + sanitize_token(object_id);
  }
  return "home_assistant_" + sanitize_token(object_id);
}

bool HomeAssistant::register_sensor(const SensorConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "sensor", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "state_topic", state_topic.c_str());
  add_string_if_not_empty(payload, "unit_of_measurement", config.unit_of_measurement);
  add_string_if_not_empty(payload, "device_class", config.device_class);
  add_string_if_not_empty(payload, "state_class", config.state_class);

  RegisteredEntity entity{
      .key = make_entity_key("sensor", object_id),
      .component = "sensor",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("sensor", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_binary_sensor(const BinarySensorConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  cJSON *payload =
      build_common_discovery_payload(snapshot, config, "binary_sensor", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "state_topic", state_topic.c_str());
  add_string_if_not_empty(payload, "device_class", config.device_class);
  add_string_if_not_empty(payload, "payload_on", config.payload_on);
  add_string_if_not_empty(payload, "payload_off", config.payload_off);

  RegisteredEntity entity{
      .key = make_entity_key("binary_sensor", object_id),
      .component = "binary_sensor",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("binary_sensor", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_button(const ButtonConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto command_topic =
      config.command_topic.empty() ? make_command_topic(object_id, "press") : config.command_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "button", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "command_topic", command_topic.c_str());
  add_string_if_not_empty(payload, "payload_press", config.payload_press);

  RegisteredEntity entity{
      .key = make_entity_key("button", object_id),
      .component = "button",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("button", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = "",
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(command_topic, config.on_command);
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_switch(const SwitchConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto command_topic =
      config.command_topic.empty() ? make_command_topic(object_id) : config.command_topic;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "switch", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "command_topic", command_topic.c_str());
  add_string_if_not_empty(payload, "state_topic", state_topic);
  add_string_if_not_empty(payload, "payload_on", config.payload_on);
  add_string_if_not_empty(payload, "payload_off", config.payload_off);
  if (config.optimistic || state_topic.empty()) {
    cJSON_AddBoolToObject(payload, "optimistic", true);
  }

  RegisteredEntity entity{
      .key = make_entity_key("switch", object_id),
      .component = "switch",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("switch", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(command_topic, config.on_command);
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_number(const NumberConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto command_topic =
      config.command_topic.empty() ? make_command_topic(object_id) : config.command_topic;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "number", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "command_topic", command_topic.c_str());
  add_string_if_not_empty(payload, "state_topic", state_topic);
  cJSON_AddNumberToObject(payload, "min", config.min_value);
  cJSON_AddNumberToObject(payload, "max", config.max_value);
  cJSON_AddNumberToObject(payload, "step", config.step);
  add_string_if_not_empty(payload, "mode", config.mode);

  RegisteredEntity entity{
      .key = make_entity_key("number", object_id),
      .component = "number",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("number", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(command_topic, config.on_command);
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_text(const TextConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto command_topic =
      config.command_topic.empty() ? make_command_topic(object_id) : config.command_topic;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "text", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "command_topic", command_topic.c_str());
  add_string_if_not_empty(payload, "state_topic", state_topic);
  cJSON_AddNumberToObject(payload, "min", config.min_length);
  cJSON_AddNumberToObject(payload, "max", config.max_length);
  add_string_if_not_empty(payload, "mode", config.mode);
  add_string_if_not_empty(payload, "pattern", config.pattern);

  RegisteredEntity entity{
      .key = make_entity_key("text", object_id),
      .component = "text",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("text", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(command_topic, config.on_command);
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_fan(const FanConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto command_topic =
      config.command_topic.empty() ? make_command_topic(object_id) : config.command_topic;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  auto percentage_command_topic = config.percentage_command_topic.empty()
                                      ? make_command_topic(object_id, "percentage/set")
                                      : config.percentage_command_topic;
  auto percentage_state_topic = config.percentage_state_topic.empty()
                                    ? make_state_topic(object_id, "percentage/state")
                                    : config.percentage_state_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "fan", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "command_topic", command_topic.c_str());
  add_string_if_not_empty(payload, "state_topic", state_topic);
  if (config.supports_percentage) {
    add_string_if_not_empty(payload, "percentage_command_topic", percentage_command_topic);
    add_string_if_not_empty(payload, "percentage_state_topic", percentage_state_topic);
  }

  RegisteredEntity entity{
      .key = make_entity_key("fan", object_id),
      .component = "fan",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("fan", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(command_topic, config.on_command);
    if (config.supports_percentage) {
      entity.command_callbacks.emplace(percentage_command_topic, config.on_command);
    }
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_cover(const CoverConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto command_topic =
      config.command_topic.empty() ? make_command_topic(object_id) : config.command_topic;
  auto state_topic = config.state_topic.empty() ? make_state_topic(object_id) : config.state_topic;
  auto position_topic = config.position_topic.empty() ? make_state_topic(object_id, "position")
                                                      : config.position_topic;
  auto set_position_topic = config.set_position_topic.empty()
                                ? make_command_topic(object_id, "position/set")
                                : config.set_position_topic;
  cJSON *payload = build_common_discovery_payload(snapshot, config, "cover", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "command_topic", command_topic.c_str());
  add_string_if_not_empty(payload, "state_topic", state_topic);
  add_string_if_not_empty(payload, "position_topic", position_topic);
  add_string_if_not_empty(payload, "set_position_topic", set_position_topic);

  RegisteredEntity entity{
      .key = make_entity_key("cover", object_id),
      .component = "cover",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("cover", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = state_topic,
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(command_topic, config.on_command);
    entity.command_callbacks.emplace(set_position_topic, config.on_command);
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

bool HomeAssistant::register_climate(const ClimateConfig &config, std::error_code &ec) {
  if (config.object_id.empty()) {
    ec = HomeAssistantErrc::invalid_argument;
    return false;
  }

  auto snapshot = get_config();
  auto object_id = sanitize_token(config.object_id);
  auto unique_id =
      config.unique_id.empty() ? default_unique_id(snapshot, object_id) : config.unique_id;
  auto mode_command_topic = config.mode_command_topic.empty()
                                ? make_command_topic(object_id, "mode/set")
                                : config.mode_command_topic;
  auto temperature_command_topic = config.temperature_command_topic.empty()
                                       ? make_command_topic(object_id, "target_temperature/set")
                                       : config.temperature_command_topic;
  auto mode_state_topic = config.mode_state_topic.empty() ? make_state_topic(object_id, "mode")
                                                          : config.mode_state_topic;
  auto temperature_state_topic = config.temperature_state_topic.empty()
                                     ? make_state_topic(object_id, "target_temperature")
                                     : config.temperature_state_topic;
  auto current_temperature_topic = config.current_temperature_topic.empty()
                                       ? make_state_topic(object_id, "current_temperature")
                                       : config.current_temperature_topic;
  cJSON *payload =
      build_common_discovery_payload(snapshot, config, "climate", object_id, unique_id);
  if (!payload) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }

  cJSON_AddStringToObject(payload, "mode_command_topic", mode_command_topic.c_str());
  cJSON_AddStringToObject(payload, "temperature_command_topic", temperature_command_topic.c_str());
  add_string_if_not_empty(payload, "mode_state_topic", mode_state_topic);
  add_string_if_not_empty(payload, "temperature_state_topic", temperature_state_topic);
  add_string_if_not_empty(payload, "current_temperature_topic", current_temperature_topic);
  cJSON *modes = cJSON_CreateArray();
  for (const auto &mode : config.modes) {
    cJSON_AddItemToArray(modes, cJSON_CreateString(mode.c_str()));
  }
  cJSON_AddItemToObject(payload, "modes", modes);

  RegisteredEntity entity{
      .key = make_entity_key("climate", object_id),
      .component = "climate",
      .object_id = object_id,
      .unique_id = unique_id,
      .discovery_topic = make_discovery_topic("climate", object_id),
      .discovery_payload = json_to_string(payload),
      .primary_state_topic = mode_state_topic,
      .command_callbacks = {},
  };
  if (config.on_command) {
    entity.command_callbacks.emplace(mode_command_topic, config.on_command);
    entity.command_callbacks.emplace(temperature_command_topic, config.on_command);
  }
  cJSON_Delete(payload);
  if (entity.discovery_payload.empty()) {
    ec = HomeAssistantErrc::json_error;
    return false;
  }
  return register_entity(std::move(entity), ec);
}

namespace {
class HomeAssistantErrorCategory : public std::error_category {
public:
  const char *name() const noexcept override { return "home_assistant"; }

  std::string message(int ev) const override {
    switch (static_cast<HomeAssistantErrc>(ev)) {
    case HomeAssistantErrc::success:
      return "success";
    case HomeAssistantErrc::invalid_configuration:
      return "invalid configuration";
    case HomeAssistantErrc::invalid_argument:
      return "invalid argument";
    case HomeAssistantErrc::already_started:
      return "already started";
    case HomeAssistantErrc::not_started:
      return "not started";
    case HomeAssistantErrc::mqtt_error:
      return "mqtt error";
    case HomeAssistantErrc::mqtt_not_connected:
      return "mqtt not connected";
    case HomeAssistantErrc::http_error:
      return "http error";
    case HomeAssistantErrc::websocket_error:
      return "websocket error";
    case HomeAssistantErrc::websocket_auth_failed:
      return "websocket authentication failed";
    case HomeAssistantErrc::json_error:
      return "json error";
    case HomeAssistantErrc::timeout:
      return "timeout";
    case HomeAssistantErrc::entity_not_found:
      return "entity not found";
    case HomeAssistantErrc::entity_already_exists:
      return "entity already exists";
    }
    return "unknown home assistant error";
  }
};
} // namespace

std::error_code espp::make_error_code(HomeAssistantErrc e) {
  static const HomeAssistantErrorCategory category;
  return {static_cast<int>(e), category};
}
