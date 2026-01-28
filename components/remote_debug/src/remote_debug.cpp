#include "remote_debug.hpp"

#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include "file_system.hpp"

using namespace espp;

namespace {
// Helper function to escape JSON strings
std::string json_escape(const std::string &str) {
  std::string escaped;
  escaped.reserve(str.size());
  for (char c : str) {
    switch (c) {
    case '\"':
      escaped += "\\\"";
      break;
    case '\\':
      escaped += "\\\\";
      break;
    case '\n':
      escaped += "\\n";
      break;
    case '\r':
      escaped += "\\r";
      break;
    case '\t':
      escaped += "\\t";
      break;
    case '\b':
      escaped += "\\b";
      break;
    case '\f':
      escaped += "\\f";
      break;
    default:
      escaped += c;
    }
  }
  return escaped;
}

// Helper to read full HTTP request body
bool read_request_body(httpd_req_t *req, std::string &buffer, size_t max_size = 4096) {
  size_t content_len = req->content_len;

  if (content_len == 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty request body");
    return false;
  }

  if (content_len > max_size) {
    httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Request too large");
    return false;
  }

  buffer.resize(content_len);
  size_t received = 0;

  while (received < content_len) {
    int ret = httpd_req_recv(req, &buffer[received], content_len - received);
    if (ret < 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        continue;
      }
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read request");
      return false;
    }
    received += ret;
  }

  return true;
}
} // anonymous namespace

RemoteDebug::RemoteDebug(const Config &config)
    : BaseComponent("RemoteDebug", config.log_level) {
  init(config);
}

RemoteDebug::~RemoteDebug() { stop(); }

void RemoteDebug::init(const Config &config) {
  config_ = config;

  // Initialize GPIO with configured mode
  for (const auto &gpio : config_.gpios) {
    gpio_mode_t init_mode = gpio.mode;

    // Promote OUTPUT to INPUT_OUTPUT for bidirectional capability
    if (init_mode == GPIO_MODE_OUTPUT) {
      init_mode = GPIO_MODE_INPUT_OUTPUT;
      logger_.debug("Promoting GPIO {} from OUTPUT to INPUT_OUTPUT", gpio.pin);
    } else if (init_mode == GPIO_MODE_OUTPUT_OD) {
      init_mode = GPIO_MODE_INPUT_OUTPUT_OD;
      logger_.debug("Promoting GPIO {} from OUTPUT_OD to INPUT_OUTPUT_OD", gpio.pin);
    }

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << gpio.pin);
    io_conf.mode = init_mode;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Store the GPIO config
    GpioConfig gpio_copy = gpio;
    gpio_copy.mode = init_mode;
    gpio_map_[gpio.pin] = gpio_copy;
    gpio_state_[gpio.pin] = gpio_get_level(gpio.pin);

    logger_.debug("Configured GPIO {} as mode {}, initial state: {}", gpio.pin, init_mode,
                  gpio_state_[gpio.pin]);
  }

  logger_.debug("Configured {} GPIOs", config_.gpios.size());

  // Calculate actual history size based on sample rate
  // adc_history_size is the number of samples we want to keep
  // Sample rate determines how many samples per second
  // So actual time = adc_history_size / samples_per_second
  float samples_per_second = 1000.0f / config_.adc_sample_rate.count();
  float actual_history_seconds = config_.adc_history_size / samples_per_second;
  logger_.info("ADC history: {} samples at {} Hz = {:.2f} seconds", config_.adc_history_size,
               samples_per_second, actual_history_seconds);

  // Initialize ADC1
  if (!config_.adc1_channels.empty()) {
    logger_.info("Initializing ADC1 with {} channels", config_.adc1_channels.size());
    std::vector<AdcConfig> adc1_configs;
    for (const auto &ch : config_.adc1_channels) {
      adc1_configs.push_back({.unit = ADC_UNIT_1, .channel = ch.channel, .attenuation = ch.atten});

      // Initialize data storage
      AdcData data;
      data.values.resize(config_.adc_history_size, 0);
      data.timestamps.resize(config_.adc_history_size, 0);
      data.channel = ch.channel;
      data.label = ch.label;
      adc1_data_.push_back(std::move(data));
    }
    adc1_ = std::make_unique<OneshotAdc>(OneshotAdc::Config{
        .unit = ADC_UNIT_1, .channels = adc1_configs, .log_level = config_.log_level});
  }

  // Initialize ADC2
  if (!config_.adc2_channels.empty()) {
    logger_.info("Initializing ADC2 with {} channels", config_.adc2_channels.size());
    std::vector<AdcConfig> adc2_configs;
    for (const auto &ch : config_.adc2_channels) {
      adc2_configs.push_back({.unit = ADC_UNIT_2, .channel = ch.channel, .attenuation = ch.atten});

      // Initialize data storage
      AdcData data;
      data.values.resize(config_.adc_history_size, 0);
      data.timestamps.resize(config_.adc_history_size, 0);
      data.channel = ch.channel;
      data.label = ch.label;
      adc2_data_.push_back(std::move(data));
    }
    adc2_ = std::make_unique<OneshotAdc>(OneshotAdc::Config{
        .unit = ADC_UNIT_2, .channels = adc2_configs, .log_level = config_.log_level});
  }

  logger_.info("RemoteDebug initialized with {} GPIOs, {} ADC1 channels, {} ADC2 channels",
               config_.gpios.size(), config_.adc1_channels.size(), config_.adc2_channels.size());
}

bool RemoteDebug::start() {
  if (is_active_) {
    logger_.warn("Already running");
    return true;
  }

  if (!start_server()) {
    return false;
  }

  // Start ADC sampling timer if we have ADCs
  if (adc1_ || adc2_) {
    logger_.info("Starting ADC sampling timer with period {} ms", config_.adc_sample_rate.count());
    sampling_active_ = true;
    adc_timer_ = std::make_unique<Timer>(Timer::Config{.name = "adc_timer",
                                                       .period = config_.adc_sample_rate,
                                                       .callback =
                                                           [this]() {
                                                             adc_sampling_task();
                                                             return false;
                                                           },
                                                       .auto_start = false,
                                                       .stack_size_bytes = config_.task_stack_size,
                                                       .priority = config_.task_priority,
                                                       .log_level = config_.log_level});
    adc_timer_->start();
  }

  // Start GPIO update timer
  if (!gpio_map_.empty()) {
    gpio_timer_ = std::make_unique<Timer>(Timer::Config{.name = "gpio_timer",
                                                        .period = config_.gpio_update_rate,
                                                        .callback =
                                                            [this]() {
                                                              gpio_update_task();
                                                              return false;
                                                            },
                                                        .auto_start = false,
                                                        .stack_size_bytes = config_.task_stack_size,
                                                        .priority = config_.task_priority,
                                                        .log_level = config_.log_level});
    gpio_timer_->start();
  }

  // Register log control endpoints
  httpd_uri_t start_log_uri = {.uri = "/api/logs/start",
                               .method = HTTP_POST,
                               .handler = start_log_handler,
                               .user_ctx = this};
  httpd_register_uri_handler(server_, &start_log_uri);

  httpd_uri_t stop_log_uri = {
      .uri = "/api/logs/stop", .method = HTTP_POST, .handler = stop_log_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &stop_log_uri);

  // Setup log redirection if enabled
  if (config_.enable_log_capture) {
    setup_log_redirection();
  }

  is_active_ = true;
  logger_.info("RemoteDebug started on port {}", config_.server_port);
  return true;
}

void RemoteDebug::stop() {
  if (!is_active_) {
    return;
  }

  sampling_active_ = false;

  // Stop timers
  if (adc_timer_) {
    adc_timer_->stop();
    adc_timer_.reset();
  }
  if (gpio_timer_) {
    gpio_timer_->stop();
    gpio_timer_.reset();
  }

  cleanup_log_redirection();
  stop_server();
  is_active_ = false;
  logger_.info("RemoteDebug stopped");
}

bool RemoteDebug::set_gpio(gpio_num_t pin, int level) {
  std::lock_guard<std::mutex> lock(gpio_mutex_);
  if (gpio_map_.find(pin) == gpio_map_.end()) {
    logger_.error("GPIO {} not configured", static_cast<int>(pin));
    return false;
  }

  // Only set if it's an output
  if (gpio_map_[pin].mode != GPIO_MODE_OUTPUT && gpio_map_[pin].mode != GPIO_MODE_OUTPUT_OD &&
      gpio_map_[pin].mode != GPIO_MODE_INPUT_OUTPUT &&
      gpio_map_[pin].mode != GPIO_MODE_INPUT_OUTPUT_OD) {
    logger_.error("GPIO {} is not configured as output", static_cast<int>(pin));
    return false;
  }

  gpio_set_level(pin, level);
  gpio_state_[pin] = level;
  return true;
}

int RemoteDebug::get_gpio(gpio_num_t pin) {
  std::lock_guard<std::mutex> lock(gpio_mutex_);
  if (gpio_map_.find(pin) == gpio_map_.end()) {
    logger_.error("GPIO {} not configured", static_cast<int>(pin));
    return -1;
  }
  return gpio_state_[pin];
}

bool RemoteDebug::configure_gpio(gpio_num_t pin, gpio_mode_t mode) {
  std::lock_guard<std::mutex> lock(gpio_mutex_);
  if (gpio_map_.find(pin) == gpio_map_.end()) {
    logger_.error("GPIO {} not configured", static_cast<int>(pin));
    return false;
  }

  logger_.info("Configuring GPIO {} to mode {}", static_cast<int>(pin), mode);

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << pin);
  io_conf.mode = mode;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

  gpio_map_[pin].mode = mode;
  gpio_state_[pin] = gpio_get_level(pin);
  return true;
}

void RemoteDebug::gpio_update_task() {
  std::lock_guard<std::mutex> lock(gpio_mutex_);
  for (auto &[pin, config] : gpio_map_) {
    gpio_state_[pin] = gpio_get_level(pin);
  }
}

void RemoteDebug::adc_sampling_task() {
  auto now = esp_timer_get_time();

  std::lock_guard<std::mutex> lock(adc_mutex_);

  // Sample ADC1
  if (adc1_) {
    auto values = adc1_->read_all_mv();
    for (size_t i = 0; i < values.size() && i < adc1_data_.size(); i++) {
      auto &data = adc1_data_[i];
      // Convert mV to V
      data.values[data.write_index] = values[i] / 1000.0f;
      data.timestamps[data.write_index] = now;
      data.write_index = (data.write_index + 1) % config_.adc_history_size;
      if (data.count < config_.adc_history_size) {
        data.count++;
      }
    }
  }

  // Sample ADC2
  if (adc2_) {
    auto values = adc2_->read_all_mv();
    for (size_t i = 0; i < values.size() && i < adc2_data_.size(); i++) {
      auto &data = adc2_data_[i];
      // Convert mV to V
      data.values[data.write_index] = values[i] / 1000.0f;
      data.timestamps[data.write_index] = now;
      data.write_index = (data.write_index + 1) % config_.adc_history_size;
      if (data.count < config_.adc_history_size) {
        data.count++;
      }
    }
  }
}

bool RemoteDebug::start_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = config_.server_port;
  config.max_uri_handlers = 8;
  config.stack_size = 8192;

  if (httpd_start(&server_, &config) != ESP_OK) {
    logger_.error("Failed to start HTTP server");
    return false;
  }

  // Register URI handlers
  httpd_uri_t root_uri = {
      .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &root_uri);

  httpd_uri_t gpio_get_uri = {
      .uri = "/api/gpio/get", .method = HTTP_GET, .handler = gpio_get_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &gpio_get_uri);

  httpd_uri_t gpio_set_uri = {
      .uri = "/api/gpio/set", .method = HTTP_POST, .handler = gpio_set_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &gpio_set_uri);

  httpd_uri_t gpio_config_uri = {.uri = "/api/gpio/config",
                                 .method = HTTP_POST,
                                 .handler = gpio_config_handler,
                                 .user_ctx = this};
  httpd_register_uri_handler(server_, &gpio_config_uri);

  httpd_uri_t adc_data_uri = {
      .uri = "/api/adc/data", .method = HTTP_GET, .handler = adc_data_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &adc_data_uri);

  httpd_uri_t logs_uri = {
      .uri = "/api/logs", .method = HTTP_GET, .handler = logs_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &logs_uri);

  return true;
}

void RemoteDebug::stop_server() {
  if (server_) {
    httpd_stop(server_);
    server_ = nullptr;
  }
}

esp_err_t RemoteDebug::root_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);
  auto html = self->generate_html();
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, html.c_str(), html.length());
  return ESP_OK;
}

esp_err_t RemoteDebug::gpio_get_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);
  auto json = self->get_gpio_state_json();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

esp_err_t RemoteDebug::gpio_set_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);

  // Read request body properly
  std::string body;
  if (!read_request_body(req, body)) {
    return ESP_FAIL;
  }

  // Parse pin=X&level=Y
  int pin = -1, level = -1;
  sscanf(body.c_str(), "pin=%d&level=%d", &pin, &level);

  if (pin < 0 || (level != 0 && level != 1)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid pin or level");
    return ESP_FAIL;
  }

  // Attempt to set GPIO
  if (!self->set_gpio(static_cast<gpio_num_t>(pin), level)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "GPIO not configured or not an output");
    return ESP_FAIL;
  }

  httpd_resp_send(req, "OK", 2);
  return ESP_OK;
}

esp_err_t RemoteDebug::gpio_config_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);

  // Read request body properly
  std::string body;
  if (!read_request_body(req, body)) {
    return ESP_FAIL;
  }

  // Parse pin=X&mode=Y
  int pin = -1, mode = -1;
  sscanf(body.c_str(), "pin=%d&mode=%d", &pin, &mode);

  // Validate mode (ESP-IDF supports modes 0-5)
  if (pin < 0 || mode < 0 || mode > 5) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid pin or mode");
    return ESP_FAIL;
  }

  // Attempt to configure GPIO
  if (!self->configure_gpio(static_cast<gpio_num_t>(pin), static_cast<gpio_mode_t>(mode))) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "GPIO configuration failed");
    return ESP_FAIL;
  }

  httpd_resp_send(req, "OK", 2);
  return ESP_OK;
}

esp_err_t RemoteDebug::adc_data_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);
  auto json = self->get_adc_data_json();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, json.c_str(), json.length());
  return ESP_OK;
}

std::string RemoteDebug::generate_html() const {
  std::stringstream ss;
  ss << R"(<!DOCTYPE html><html><head><title>Remote Debug - )" << config_.device_name << R"(</title>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
       background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); 
       min-height: 100vh; padding: 20px; }
.container { max-width: 1200px; margin: 0 auto; }
h1 { color: white; text-align: center; margin-bottom: 30px; font-size: 2.5em; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
.card { background: white; border-radius: 15px; padding: 25px; margin-bottom: 25px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }
h2 { color: #667eea; margin-bottom: 20px; font-size: 1.8em; border-bottom: 3px solid #667eea; padding-bottom: 10px; }
.gpio { display: flex; align-items: center; justify-content: space-between; padding: 15px; 
        margin: 10px 0; background: #f8f9fa; border-radius: 10px; transition: all 0.3s; }
.gpio:hover { background: #e9ecef; transform: translateX(5px); }
.gpio-label { font-weight: 600; color: #495057; font-size: 1.1em; }
.gpio-controls { display: flex; gap: 10px; align-items: center; }
.gpio-state { font-weight: bold; min-width: 60px; text-align: center; padding: 5px 10px; 
              border-radius: 5px; background: #dee2e6; }
button { padding: 10px 20px; border: none; border-radius: 8px; cursor: pointer; 
         font-weight: 600; font-size: 1em; transition: all 0.3s; }
button:hover { transform: translateY(-2px); box-shadow: 0 5px 15px rgba(0,0,0,0.2); }
.btn-high { background: linear-gradient(135deg, #48c774 0%, #39a561 100%); color: white; }
.btn-low { background: linear-gradient(135deg, #f14668 0%, #d92840 100%); color: white; }
.adc { display: flex; justify-content: space-between; align-items: center; padding: 15px; 
       margin: 10px 0; background: #f8f9fa; border-radius: 10px; }
.adc-label { font-weight: 600; color: #495057; font-size: 1.1em; }
.adc-value { font-size: 1.3em; font-weight: bold; color: #667eea; }
#chart { width: 100%; height: 400px; border-radius: 10px; background: #f8f9fa; }
</style></head><body>
<div class='container'>
<h1>&#x1F527; )"
     << config_.device_name << R"(</h1>)";

  // GPIO controls
  ss << "<div class='card'><h2>GPIO Control</h2>";
  ss << "<div style='display: grid; grid-template-columns: 2fr 1fr 1fr 1fr; gap: 10px; padding: "
        "10px; "
        "font-weight: bold; border-bottom: 2px solid #667eea; margin-bottom: 15px;'>";
  ss << "<div>Pin</div><div style='text-align: center;'>Mode</div><div style='text-align: "
        "center;'>Controls</div><div style='text-align: "
        "center;'>State</div>";
  ss << "</div>";
  for (const auto &[pin, cfg] : gpio_map_) {
    ss << "<div style='display: grid; grid-template-columns: 2fr 1fr 1fr 1fr; gap: 10px; padding: "
          "15px; margin: 10px 0; background: #f8f9fa; border-radius: 10px; align-items: center;'>";
    ss << "<span class='gpio-label'>"
       << (cfg.label.empty() ? "GPIO " + std::to_string(pin) : cfg.label) << " (GPIO " << pin
       << ")</span>";
    ss << "<select id='mode" << pin << "' onchange='setMode(" << pin
       << ",this.value)' style='padding: 8px; border-radius: 5px; border: 2px solid #667eea;'>";
    ss << "<option value='1'" << (cfg.mode == GPIO_MODE_INPUT ? " selected" : "")
       << ">Input</option>";
    ss << "<option value='3'" << (cfg.mode == GPIO_MODE_INPUT_OUTPUT ? " selected" : "")
       << ">Output</option>";
    ss << "</select>";
    ss << "<div style='display: flex; gap: 10px; justify-content: center;' id='controls" << pin
       << "'>";
    ss << "<button class='btn-high' onclick='setGpio(" << pin << ",1)'>HIGH</button>";
    ss << "<button class='btn-low' onclick='setGpio(" << pin << ",0)'>LOW</button>";
    ss << "</div>";
    ss << "<span class='gpio-state' id='gpio" << pin << "' style='text-align: center;'>?</span>";
    ss << "</div>";
  }
  ss << "</div>";

  // ADC display
  if (!adc1_data_.empty() || !adc2_data_.empty()) {
    ss << "<div class='card'><h2>ADC Monitoring</h2>";
    for (const auto &data : adc1_data_) {
      ss << "<div class='adc'>";
      ss << "<span class='adc-label'>"
         << (data.label.empty()
                 ? ("ADC1_CH" + std::to_string(static_cast<int>(data.channel)))
                 : data.label + " (ADC1_CH" + std::to_string(static_cast<int>(data.channel)) + ")")
         << "</span>";
      ss << "<span class='adc-value' id='adc1_" << static_cast<int>(data.channel) << "'>?</span>";
      ss << "</div>";
    }
    for (const auto &data : adc2_data_) {
      ss << "<div class='adc'>";
      ss << "<span class='adc-label'>"
         << (data.label.empty()
                 ? ("ADC2_CH" + std::to_string(static_cast<int>(data.channel)))
                 : data.label + " (ADC2_CH" + std::to_string(static_cast<int>(data.channel)) + ")")
         << "</span>";
      ss << "<span class='adc-value' id='adc2_" << static_cast<int>(data.channel) << "'>?</span>";
      ss << "</div>";
    }
    ss << "<canvas id='chart'></canvas>";
    ss << "</div>";
  }

  // Log viewer
  if (config_.enable_log_capture) {
    ss << R"(<div class='card'>
<h2>Console Logs</h2>
<div style='margin-bottom: 10px;'>
<button class='btn-high' onclick='startLogs()'>Start Logging</button>
<button class='btn-low' onclick='stopLogs()'>Stop Logging</button>
</div>
<div style='background: #1e1e1e; color: #d4d4d4; padding: 15px; border-radius: 8px; font-family: "Courier New", monospace; font-size: 12px; max-height: 400px; overflow-y: auto;'>
<div id='logBox' style='margin: 0; white-space: pre-wrap; word-wrap: break-word;'>Loading logs...</div>
</div>
</div>)";
  }

  // JavaScript
  ss << R"(<script>
const adcHistory = {};
const colors = ['#667eea', '#764ba2', '#48c774', '#f14668', '#ffdd57', '#3298dc'];
let colorIndex = 0;

function setMode(pin, mode) {
  fetch('/api/gpio/config', {method: 'POST', body: 'pin=' + pin + '&mode=' + mode})
    .then(r => r.text())
    .then(() => {
      updateControlsVisibility();
      updateGpio();
    })
    .catch(err => console.error('Failed to set mode:', err));
}

function startLogs() {
  fetch('/api/logs/start', {method: 'POST'})
    .then(r => r.text())
    .then(() => console.log('Logging started'))
    .catch(err => console.error('Failed to start logs:', err));
}

function stopLogs() {
  fetch('/api/logs/stop', {method: 'POST'})
    .then(r => r.text())
    .then(() => console.log('Logging stopped'))
    .catch(err => console.error('Failed to stop logs:', err));
}

function setGpio(pin, level) {
  fetch('/api/gpio/set', {method: 'POST', body: 'pin=' + pin + '&level=' + level})
    .then(r => r.text())
    .then(() => updateGpio());
}

function updateGpio() {
  fetch('/api/gpio/get')
    .then(r => r.json())
    .then(data => {
      for (let pin in data) {
        const elem = document.getElementById('gpio' + pin);
        if (elem) {
          elem.innerText = data[pin] ? 'HIGH' : 'LOW';
          elem.style.background = data[pin] ? '#48c774' : '#f14668';
          elem.style.color = 'white';
        }
      }
    });
}

function updateControlsVisibility() {
  )"
     << "[";
  bool first = true;
  for (const auto &[pin, cfg] : gpio_map_) {
    if (!first)
      ss << ",";
    ss << pin;
    first = false;
  }
  ss << R"(].forEach(pin => {
    const modeSelect = document.getElementById('mode' + pin);
    const controls = document.getElementById('controls' + pin);
    if (modeSelect && controls) {
      // Show controls for input_output (3), hide for input (1)
      controls.style.visibility = (modeSelect.value == '3') ? 'visible' : 'hidden';
    }
  });
}

function updateAdc() {
  fetch('/api/adc/data')
    .then(r => r.json())
    .then(data => {
      for (let key in data) {
        const adcData = data[key];
        const unit = adcData.unit;
        const channel = adcData.channel;
        
        // Update the ADC value display using unit and channel
        const elem = document.getElementById(`adc${unit}_${channel}`);
        if (elem) {
          const volts = adcData.voltage.toFixed(3);
          elem.innerText = volts + ' V';
        }
        
        if (!adcHistory[key]) {
          adcHistory[key] = { 
            data: [], // Store [timestamp, value] pairs
            color: colors[colorIndex++ % colors.length], 
            label: adcData.label
          };
        }
        
        // Get batched history data - replace the entire dataset
        const history = adcData.history;
        if (history && history.values && history.timestamps && history.values.length > 0) {
          // Replace the entire dataset with fresh data from the ring buffer
          adcHistory[key].data = [];
          for (let i = 0; i < history.values.length; i++) {
            adcHistory[key].data.push([history.timestamps[i], history.values[i]]);
          }
        }
      }
      drawChart();
    });
}

function drawChart() {
  const canvas = document.getElementById('chart');
  if (!canvas) return;
  
  const ctx = canvas.getContext('2d');
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height;
  
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  const padding = 40;
  const chartWidth = canvas.width - 2 * padding;
  const chartHeight = canvas.height - 2 * padding;
  
  // Find min/max across all series (values are in volts)
  let minVal = Infinity, maxVal = -Infinity;
  for (let key in adcHistory) {
    const data = adcHistory[key].data;
    if (data.length > 0) {
      for (const [_, value] of data) {
        minVal = Math.min(minVal, value);
        maxVal = Math.max(maxVal, value);
      }
    }
  }
  
  if (minVal === Infinity) return;
  
  const range = maxVal - minVal || 1;
  
  // Draw grid
  ctx.strokeStyle = '#e0e0e0';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = padding + (chartHeight * i / 4);
    ctx.beginPath();
    ctx.moveTo(padding, y);
    ctx.lineTo(padding + chartWidth, y);
    ctx.stroke();
    
    const val = (maxVal - (range * i / 4)).toFixed(2);
    ctx.fillStyle = '#666';
    ctx.font = '12px Arial';
    ctx.fillText(val + 'V', 5, y + 4);
  }
  
  // Find time range across all data
  let minTime = Infinity, maxTime = -Infinity;
  for (let key in adcHistory) {
    const data = adcHistory[key].data;
    if (data.length > 0) {
      minTime = Math.min(minTime, data[0][0]);
      maxTime = Math.max(maxTime, data[data.length - 1][0]);
    }
  }
  const timeRange = maxTime - minTime || 1;
  
  // Draw each ADC line
  for (let key in adcHistory) {
    const history = adcHistory[key];
    if (history.data.length < 2) continue;
    
    ctx.strokeStyle = history.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    for (let i = 0; i < history.data.length; i++) {
      const [timestamp, value] = history.data[i];
      const x = padding + (chartWidth * (timestamp - minTime) / timeRange);
      const y = padding + chartHeight - ((value - minVal) / range * chartHeight);
      
      if (i === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
  }
  
  // Draw legend
  let legendY = padding + 10;
  for (let key in adcHistory) {
    const label = adcHistory[key].label || ('ADC ' + key);
    ctx.fillStyle = adcHistory[key].color;
    ctx.fillRect(padding + chartWidth - 150, legendY, 15, 15);
    ctx.fillStyle = '#333';
    ctx.font = '12px Arial';
    ctx.fillText(label, padding + chartWidth - 130, legendY + 12);
    legendY += 20;
  }
}

// Initialize on page load
updateControlsVisibility();
updateGpio();
updateAdc();

// Update periodically
setInterval(updateGpio, 200);
setInterval(updateAdc, 100);

// Update logs if enabled
)";

  if (config_.enable_log_capture) {
    ss << R"(
// ANSI color parser
function parseAnsi(text) {
    const ansiRegex = /\x1b\[([0-9;]+)m/g;
    const colorMap = {
        '0': 'reset',
        '30': '#000000', '31': '#e74c3c', '32': '#2ecc71', '33': '#f39c12',
        '34': '#3498db', '35': '#9b59b6', '36': '#1abc9c', '37': '#ecf0f1',
        '90': '#7f8c8d', '91': '#ff6b6b', '92': '#51cf66', '93': '#ffd43b',
        '94': '#74c0fc', '95': '#da77f2', '96': '#4dabf7', '97': '#f8f9fa'
    };
    
    let result = '';
    let lastIndex = 0;
    let currentColor = '#d4d4d4';
    let currentBold = false;
    
    text = text.replace(/\r?\n/g, '\n');
    
    let match;
    while ((match = ansiRegex.exec(text)) !== null) {
        // Add text before the escape sequence
        if (match.index > lastIndex) {
            const textBefore = text.substring(lastIndex, match.index);
            if (textBefore) {
                result += `<span style="color:${currentColor};${currentBold ? 'font-weight:bold;' : ''}">${textBefore.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</span>`;
            }
        }
        
        // Parse the escape code
        const codes = match[1].split(';').map(c => c.trim());
        for (const code of codes) {
            if (code === '0') {
                currentColor = '#d4d4d4';
                currentBold = false;
            } else if (code === '1') {
                currentBold = true;
            } else if (colorMap[code]) {
                currentColor = colorMap[code];
            }
        }
        
        lastIndex = match.index + match[0].length;
    }
    
    // Add remaining text
    if (lastIndex < text.length) {
        const remaining = text.substring(lastIndex);
        result += `<span style="color:${currentColor};${currentBold ? 'font-weight:bold;' : ''}">${remaining.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</span>`;
    }
    
    return result;
}

setInterval(() => {
    fetch('/api/logs')
        .then(r => r.text())
        .then(data => {
            const logBox = document.getElementById('logBox');
            logBox.innerHTML = parseAnsi(data);
            logBox.scrollTop = logBox.scrollHeight;
        });
}, 1000);
)";
  }

  ss << R"(
</script></div></body></html>)";

  return ss.str();
}

std::string RemoteDebug::get_gpio_state_json() const {
  std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(gpio_mutex_));
  std::stringstream ss;
  ss << "{";
  bool first = true;
  for (const auto &[pin, cfg] : gpio_map_) {
    if (!first)
      ss << ",";
    ss << "\"" << static_cast<int>(pin) << "\":" << gpio_state_.at(pin);
    first = false;
  }
  ss << "}";
  return ss.str();
}

std::string RemoteDebug::get_adc_data_json() const {
  std::lock_guard<std::mutex> lock(adc_mutex_);
  std::stringstream ss;
  ss << "{";

  bool first = true;

  // Process ADC1 channels
  for (size_t i = 0; i < adc1_data_.size(); i++) {
    const auto &data = adc1_data_[i];
    if (!first)
      ss << ",";

    // Use unique key based on unit and channel (not label which can be non-unique)
    std::string key = fmt::format("ADC1_{}", static_cast<int>(data.channel));

    // Get the most recent value
    float voltage = 0.0f;
    if (data.count > 0) {
      size_t latest_idx = (data.write_index + data.values.size() - 1) % data.values.size();
      voltage = data.values[latest_idx];
    }

    ss << "\"" << key << "\":{";
    ss << "\"voltage\":" << std::fixed << std::setprecision(3) << voltage << ",";
    ss << "\"current\":" << voltage << ",";
    ss << "\"label\":\"" << json_escape(data.label) << "\",";
    ss << "\"unit\":1,";
    ss << "\"channel\":" << static_cast<int>(data.channel) << ",";

    // Send all buffered data for plotting
    ss << "\"history\":{";
    ss << "\"count\":" << data.count << ",";
    ss << "\"values\":[";

    // Send data in chronological order (oldest to newest)
    const size_t count = data.count;
    const size_t capacity = data.values.size();
    for (size_t j = 0; j < count; j++) {
      if (j > 0)
        ss << ",";
      size_t idx = (data.write_index + capacity - count + j) % capacity;
      ss << std::fixed << std::setprecision(3) << data.values[idx];
    }
    ss << "],";

    ss << "\"timestamps\":[";
    for (size_t j = 0; j < count; j++) {
      if (j > 0)
        ss << ",";
      size_t idx = (data.write_index + capacity - count + j) % capacity;
      ss << data.timestamps[idx];
    }
    ss << "]";
    ss << "}"; // close history
    ss << "}"; // close channel
    first = false;
  }

  // Process ADC2 channels
  for (size_t i = 0; i < adc2_data_.size(); i++) {
    const auto &data = adc2_data_[i];
    if (!first)
      ss << ",";

    // Use unique key based on unit and channel (not label which can be non-unique)
    std::string key = fmt::format("ADC2_{}", static_cast<int>(data.channel));

    // Get the most recent value
    float voltage = 0.0f;
    if (data.count > 0) {
      size_t latest_idx = (data.write_index + data.values.size() - 1) % data.values.size();
      voltage = data.values[latest_idx];
    }

    ss << "\"" << key << "\":{";
    ss << "\"voltage\":" << std::fixed << std::setprecision(3) << voltage << ",";
    ss << "\"current\":" << voltage << ",";
    ss << "\"label\":\"" << json_escape(data.label) << "\",";
    ss << "\"unit\":2,";
    ss << "\"channel\":" << static_cast<int>(data.channel) << ",";

    // Send all buffered data for plotting
    ss << "\"history\":{";
    ss << "\"count\":" << data.count << ",";
    ss << "\"values\":[";

    // Send data in chronological order (oldest to newest)
    const size_t count = data.count;
    const size_t capacity = data.values.size();
    for (size_t j = 0; j < count; j++) {
      if (j > 0)
        ss << ",";
      size_t idx = (data.write_index + capacity - count + j) % capacity;
      ss << std::fixed << std::setprecision(3) << data.values[idx];
    }
    ss << "],";

    ss << "\"timestamps\":[";
    for (size_t j = 0; j < count; j++) {
      if (j > 0)
        ss << ",";
      size_t idx = (data.write_index + capacity - count + j) % capacity;
      ss << data.timestamps[idx];
    }
    ss << "]";
    ss << "}"; // close history
    ss << "}"; // close channel
    first = false;
  }

  ss << "}";
  return ss.str();
}

void RemoteDebug::setup_log_redirection() {
  if (!config_.enable_log_capture) {
    logger_.debug("Log capture not enabled");
    return;
  }

  if (log_file_) {
    logger_.warn("Stdout already redirected to log file");
    return;
  }

#ifndef CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE
  logger_.warn("**************************************************************");
  logger_.warn("WARNING: CONFIG_LITTLEFS_FLUSH_FILE_EVERY_WRITE is not enabled!");
  logger_.warn("Logs will not appear in real-time on the web interface.");
  logger_.warn("Enable this option in menuconfig:");
  logger_.warn("  Component config -> LittleFS -> Flush file every write");
  logger_.warn("**************************************************************");
#endif

  auto &fs = FileSystem::get();
  auto log_path = fs.get_root_path() / config_.log_file_path;

  logger_.info("Attempting to redirect stdout to: {}", log_path);
  logger_.info("Log buffer size: {} bytes", config_.max_log_size);

  // Redirect stdout using freopen
  FILE *result = freopen(log_path.string().c_str(), "w", stdout);
  if (!result) {
    logger_.error("Failed to redirect stdout to log file: {} (errno: {})", log_path,
                  strerror(errno));
    return;
  }

  log_file_ = result;

  // remove file buffer so logs are written immediately
  setvbuf(log_file_, nullptr, _IONBF, 0);

  // Write test message directly to stdout (which is now the file)
  fmt::print("=== Log capture started at {} s ===\n", Logger::get_time());
  fmt::print("Remote Debug log file: {}\n", log_path);
  fflush(stdout);
}

void RemoteDebug::stop_log_redirection() {
  if (!log_file_) {
    logger_.warn("Stdout not redirected to log file");
    return;
  }

  // Write final message
  fmt::print("=== Log capture stopped at {} s ===\n", Logger::get_time());
  fflush(stdout);

  // Redirect stdout back to console
  FILE *result = freopen("/dev/console", "w", stdout);
  if (!result) {
    // Can't log this error since stdout is broken
    return;
  }

  log_file_ = nullptr;
  logger_.info("Successfully restored stdout to console");
  // we have to suppress the resource leak warning here because freopen returns
  // a new FILE* that we don't own and shouldn't fclose since this is just a
  // virtual file system. If we called fclose here, it would close stdout, which
  // would then lead to no console output and potential crashes next time we
  // tried to redirect stdout to file.
} // cppcheck-suppress resourceLeak

void RemoteDebug::cleanup_log_redirection() { stop_log_redirection(); }

std::string RemoteDebug::get_logs() const {
  if (!config_.enable_log_capture) {
    return "Log capture not enabled";
  }

  // Flush stdout to ensure all logs are written
  if (log_file_) {
    fflush(log_file_);
  }

  auto &fs = FileSystem::get();
  auto log_path = fs.get_root_path() / config_.log_file_path;

  // Open a separate file handle for reading
  FILE *read_file = fopen(log_path.string().c_str(), "r");
  if (!read_file) {
    return fmt::format("Failed to open log file for reading (errno: {})", strerror(errno));
  }

  // Get file size
  fseek(read_file, 0, SEEK_END);
  long file_size = ftell(read_file);

  if (file_size <= 0) {
    fclose(read_file);
    return "Log file is empty";
  }

  // Limit to max_log_size and seek to start of content we want to read
  long read_size = std::min(file_size, static_cast<long>(config_.max_log_size));
  long start_pos = file_size - read_size;
  fseek(read_file, start_pos, SEEK_SET);

  // Read content
  std::string content;
  content.resize(read_size);
  size_t bytes_read = fread(&content[0], 1, read_size, read_file);
  content.resize(bytes_read);
  fclose(read_file);

  if (bytes_read == 0) {
    return fmt::format("Failed to read log file (errno: {}) (read_size: {})", strerror(errno),
                       read_size);
  }

  return content;
}

esp_err_t RemoteDebug::logs_handler(httpd_req_t *req) {
  RemoteDebug *self = static_cast<RemoteDebug *>(req->user_ctx);

  std::string logs = self->get_logs();

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, logs.c_str(), logs.length());
  return ESP_OK;
}

esp_err_t RemoteDebug::start_log_handler(httpd_req_t *req) {
  RemoteDebug *self = static_cast<RemoteDebug *>(req->user_ctx);

  if (!self->config_.enable_log_capture) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Log capture not enabled");
    return ESP_FAIL;
  }

  self->setup_log_redirection();
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

esp_err_t RemoteDebug::stop_log_handler(httpd_req_t *req) {
  RemoteDebug *self = static_cast<RemoteDebug *>(req->user_ctx);

  if (!self->config_.enable_log_capture) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Log capture not enabled");
    return ESP_FAIL;
  }

  self->stop_log_redirection();
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}
