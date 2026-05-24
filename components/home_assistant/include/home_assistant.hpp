#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "base_component.hpp"
#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_websocket_client.h"
#include "mqtt_client.h"

namespace espp {

enum class HomeAssistantErrc {
  success = 0,
  invalid_configuration,
  invalid_argument,
  already_started,
  not_started,
  mqtt_error,
  mqtt_not_connected,
  http_error,
  websocket_error,
  websocket_auth_failed,
  json_error,
  timeout,
  entity_not_found,
  entity_already_exists,
};

std::error_code make_error_code(HomeAssistantErrc e);

class HomeAssistant : public BaseComponent {
public:
  using CommandCallback = std::function<void(std::string_view topic, std::string_view payload)>;
  using WebsocketMessageCallback = std::function<void(std::string_view json)>;

  struct DeviceConfig {
    std::string name;
    std::vector<std::string> identifiers;
    std::string manufacturer;
    std::string model;
    std::string sw_version;
    std::string hw_version;
    std::string suggested_area;
    std::string configuration_url;
  };

  struct MqttConfig {
    std::string broker_uri;
    std::string username;
    std::string password;
    std::string client_id;
    std::string discovery_prefix{"homeassistant"};
    std::string node_id;
    std::string state_prefix{"espp/home_assistant"};
    std::string availability_topic;
    int qos{0};
    bool retain_state{false};
    bool publish_availability{true};
    bool auto_reconnect{true};
    bool use_crt_bundle{true};
    int keepalive_seconds{120};
    int network_timeout_ms{10000};
    int buffer_size{1024};
    int task_stack_size{6144};
  };

  struct ApiConfig {
    std::string base_url;
    std::string access_token;
    std::string websocket_url;
    bool enable_websocket{false};
    bool use_crt_bundle{true};
    bool skip_cert_common_name_check{false};
    int timeout_ms{10000};
    int websocket_task_stack_size{6144};
    int websocket_buffer_size{1024};
    int websocket_ping_interval_sec{10};
    WebsocketMessageCallback on_websocket_message{nullptr};
  };

  struct Config {
    DeviceConfig device;
    MqttConfig mqtt;
    ApiConfig api;
    std::string availability_payload_online{"online"};
    std::string availability_payload_offline{"offline"};
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  struct EntityConfig {
    std::string object_id;
    std::string name;
    std::string unique_id;
    std::string icon;
    std::string entity_category;
    std::string availability_topic;
    std::string json_attributes_topic;
    bool enabled_by_default{true};
  };

  struct SensorConfig : EntityConfig {
    std::string state_topic;
    std::string unit_of_measurement;
    std::string device_class;
    std::string state_class;
  };

  struct BinarySensorConfig : EntityConfig {
    std::string state_topic;
    std::string payload_on{"ON"};
    std::string payload_off{"OFF"};
    std::string device_class;
  };

  struct ButtonConfig : EntityConfig {
    std::string command_topic;
    std::string payload_press{"PRESS"};
    CommandCallback on_command{nullptr};
  };

  struct SwitchConfig : EntityConfig {
    std::string command_topic;
    std::string state_topic;
    std::string payload_on{"ON"};
    std::string payload_off{"OFF"};
    bool optimistic{false};
    CommandCallback on_command{nullptr};
  };

  struct NumberConfig : EntityConfig {
    std::string command_topic;
    std::string state_topic;
    float min_value{0.0f};
    float max_value{100.0f};
    float step{1.0f};
    std::string mode{"auto"};
    CommandCallback on_command{nullptr};
  };

  struct TextConfig : EntityConfig {
    std::string command_topic;
    std::string state_topic;
    int min_length{0};
    int max_length{255};
    std::string mode{"text"};
    std::string pattern;
    CommandCallback on_command{nullptr};
  };

  struct FanConfig : EntityConfig {
    std::string command_topic;
    std::string state_topic;
    std::string percentage_command_topic;
    std::string percentage_state_topic;
    bool supports_percentage{false};
    CommandCallback on_command{nullptr};
  };

  struct CoverConfig : EntityConfig {
    std::string command_topic;
    std::string state_topic;
    std::string position_topic;
    std::string set_position_topic;
    CommandCallback on_command{nullptr};
  };

  struct ClimateConfig : EntityConfig {
    std::string mode_command_topic;
    std::string temperature_command_topic;
    std::string mode_state_topic;
    std::string temperature_state_topic;
    std::string current_temperature_topic;
    std::vector<std::string> modes{"off", "heat", "cool", "auto"};
    CommandCallback on_command{nullptr};
  };

  explicit HomeAssistant(const Config &config);
  ~HomeAssistant();

  void set_config(const Config &config);
  Config get_config() const;

  bool start(std::error_code &ec);
  void stop();

  bool is_mqtt_connected() const { return mqtt_connected_; }
  bool is_websocket_connected() const { return websocket_authenticated_; }

  bool connect_websocket(std::error_code &ec);
  void disconnect_websocket();
  bool send_websocket_command(std::string_view json, std::error_code &ec);
  bool subscribe_websocket_events(std::string_view event_type, uint32_t &subscription_id,
                                  std::error_code &ec);

  bool get_config(std::string &json, std::error_code &ec);
  bool get_state(std::string_view entity_id, std::string &json, std::error_code &ec);
  bool set_state(std::string_view entity_id, std::string_view state,
                 std::string_view attributes_json, std::string &json, std::error_code &ec);
  bool call_service(std::string_view domain, std::string_view service,
                    std::string_view service_data_json, std::string &json, std::error_code &ec);

  bool register_sensor(const SensorConfig &config, std::error_code &ec);
  bool register_binary_sensor(const BinarySensorConfig &config, std::error_code &ec);
  bool register_button(const ButtonConfig &config, std::error_code &ec);
  bool register_switch(const SwitchConfig &config, std::error_code &ec);
  bool register_number(const NumberConfig &config, std::error_code &ec);
  bool register_text(const TextConfig &config, std::error_code &ec);
  bool register_fan(const FanConfig &config, std::error_code &ec);
  bool register_cover(const CoverConfig &config, std::error_code &ec);
  bool register_climate(const ClimateConfig &config, std::error_code &ec);
  bool remove_entity(std::string_view component, std::string_view object_id, std::error_code &ec);

  bool publish_state(std::string_view topic, std::string_view state, std::error_code &ec,
                     bool retain = false);
  bool publish_entity_state(std::string_view object_id, std::string_view state, std::error_code &ec,
                            bool retain = false);

  std::string make_state_topic(std::string_view object_id, std::string_view suffix = "state") const;
  std::string make_command_topic(std::string_view object_id, std::string_view suffix = "set") const;
  std::string make_discovery_topic(std::string_view component, std::string_view object_id) const;

protected:
  struct RegisteredEntity {
    std::string key;
    std::string component;
    std::string object_id;
    std::string unique_id;
    std::string discovery_topic;
    std::string discovery_payload;
    std::string primary_state_topic;
    std::unordered_map<std::string, CommandCallback> command_callbacks;
  };

  static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                                 void *event_data);
  static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                                      void *event_data);

  bool start_mqtt(std::error_code &ec);
  void stop_mqtt();
  bool perform_http_request(esp_http_client_method_t method, std::string_view path,
                            std::string_view body, std::string &response, std::error_code &ec);
  bool send_websocket_text(std::string_view json, std::error_code &ec, bool require_auth);
  void handle_websocket_message(std::string_view json);
  void handle_mqtt_command(std::string_view topic, std::string_view payload);
  bool publish_discovery(const RegisteredEntity &entity, std::error_code &ec);
  bool publish_availability(std::string_view payload, std::error_code &ec);
  bool register_entity(RegisteredEntity entity, std::error_code &ec);

  Config get_config_locked() const;

  mutable std::mutex config_mutex_;
  Config config_;

  mutable std::mutex entity_mutex_;
  std::unordered_map<std::string, RegisteredEntity> entities_;
  std::unordered_map<std::string, std::string> object_lookup_;

  mutable std::mutex websocket_mutex_;
  std::string websocket_buffer_;
  std::error_code websocket_last_error_;
  std::string websocket_last_error_message_;

  esp_mqtt_client_handle_t mqtt_client_{nullptr};
  esp_websocket_client_handle_t websocket_client_{nullptr};

  std::atomic<bool> started_{false};
  std::atomic<bool> mqtt_connected_{false};
  std::atomic<bool> websocket_transport_connected_{false};
  std::atomic<bool> websocket_authenticated_{false};
  std::atomic<uint32_t> websocket_next_id_{1};
};

} // namespace espp

namespace std {
template <> struct is_error_code_enum<espp::HomeAssistantErrc> : true_type {};
} // namespace std
