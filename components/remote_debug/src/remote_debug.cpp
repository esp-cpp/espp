#include "remote_debug.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace espp;

RemoteDebug::RemoteDebug(const Config &config)
    : BaseComponent("RemoteDebug", config.log_level) {
  init(config);
}

RemoteDebug::~RemoteDebug() { stop(); }

void RemoteDebug::init(const Config &config) {
  config_ = config;

  // Initialize GPIO - default to input mode
  for (const auto &gpio : config_.gpios) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << gpio.pin);
    io_conf.mode = GPIO_MODE_INPUT; // Default to input
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Store the GPIO config with input mode
    GpioConfig gpio_copy = gpio;
    gpio_copy.mode = GPIO_MODE_INPUT;
    gpio_map_[gpio.pin] = gpio_copy;
    gpio_state_[gpio.pin] = gpio_get_level(gpio.pin);

    logger_.debug("Configured GPIO {} as input, initial state: {}", gpio.pin,
                  gpio_state_[gpio.pin]);
  }

  // Initialize ADC1
  if (!config_.adc1_channels.empty()) {
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

  // Start ADC sampling task if we have ADCs
  if (adc1_ || adc2_) {
    sampling_active_ = true;
    sampling_thread_ = std::make_unique<std::thread>([this]() { adc_sampling_task(); });
  }

  // Start GPIO update task
  if (!gpio_map_.empty()) {
    gpio_thread_ = std::make_unique<std::thread>([this]() { gpio_update_task(); });
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
  if (sampling_thread_ && sampling_thread_->joinable()) {
    sampling_thread_->join();
  }
  if (gpio_thread_ && gpio_thread_->joinable()) {
    gpio_thread_->join();
  }

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
  while (sampling_active_) {
    {
      std::lock_guard<std::mutex> lock(gpio_mutex_);
      for (auto &[pin, config] : gpio_map_) {
        gpio_state_[pin] = gpio_get_level(pin);
      }
    }
    std::this_thread::sleep_for(config_.gpio_update_rate);
  }
}

void RemoteDebug::adc_sampling_task() {
  while (sampling_active_) {
    auto now = esp_timer_get_time();

    std::lock_guard<std::mutex> lock(adc_mutex_);

    // Sample ADC1
    if (adc1_) {
      auto values = adc1_->read_all_mv();
      for (size_t i = 0; i < values.size() && i < adc1_data_.size(); i++) {
        auto &data = adc1_data_[i];
        data.values[data.write_index] = values[i];
        data.timestamps[data.write_index] = now;
        data.write_index = (data.write_index + 1) % config_.adc_history_size;
      }
    }

    // Sample ADC2
    if (adc2_) {
      auto values = adc2_->read_all_mv();
      for (size_t i = 0; i < values.size() && i < adc2_data_.size(); i++) {
        auto &data = adc2_data_[i];
        data.values[data.write_index] = values[i];
        data.timestamps[data.write_index] = now;
        data.write_index = (data.write_index + 1) % config_.adc_history_size;
      }
    }

    std::this_thread::sleep_for(config_.adc_sample_rate);
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

  // Parse POST data
  char buf[100];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  // Parse pin=X&level=Y
  int pin = -1, level = -1;
  sscanf(buf, "pin=%d&level=%d", &pin, &level);

  if (pin >= 0 && (level == 0 || level == 1)) {
    self->set_gpio(static_cast<gpio_num_t>(pin), level);
    httpd_resp_send(req, "OK", 2);
  } else {
    httpd_resp_send_500(req);
  }

  return ESP_OK;
}

esp_err_t RemoteDebug::gpio_config_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);

  // Parse POST data
  char buf[100];
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  buf[ret] = '\0';

  // Parse pin=X&mode=Y
  int pin = -1, mode = -1;
  sscanf(buf, "pin=%d&mode=%d", &pin, &mode);

  if (pin >= 0 && mode >= 0 && mode <= 6) {
    self->configure_gpio(static_cast<gpio_num_t>(pin), static_cast<gpio_mode_t>(mode));
    httpd_resp_send(req, "OK", 2);
  } else {
    httpd_resp_send_500(req);
  }

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
      const now = Date.now();
      for (let key in data) {
        // key format is "1_8" for ADC1 channel 8
        const elem = document.getElementById('adc' + key);
        if (elem) {
          const volts = (data[key].voltage / 1000.0).toFixed(3);
          elem.innerText = volts + ' V';
        }
        
        if (!adcHistory[key]) {
          adcHistory[key] = { values: [], times: [], color: colors[colorIndex++ % colors.length], label: data[key].label };
        }
        
        adcHistory[key].values.push(data[key].voltage / 1000.0); // Store in volts
        adcHistory[key].times.push(now);
        
        // Keep last 100 samples
        if (adcHistory[key].values.length > 100) {
          adcHistory[key].values.shift();
          adcHistory[key].times.shift();
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
    const vals = adcHistory[key].values;
    if (vals.length > 0) {
      minVal = Math.min(minVal, ...vals);
      maxVal = Math.max(maxVal, ...vals);
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
  
  // Draw each ADC line
  for (let key in adcHistory) {
    const history = adcHistory[key];
    if (history.values.length < 2) continue;
    
    ctx.strokeStyle = history.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    for (let i = 0; i < history.values.length; i++) {
      const x = padding + (chartWidth * i / (history.values.length - 1));
      const y = padding + chartHeight - ((history.values[i] - minVal) / range * chartHeight);
      
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
setInterval(updateGpio, 500);
setInterval(updateAdc, 200);
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
  std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(adc_mutex_));
  std::stringstream ss;
  ss << "{";

  bool first = true;

  // ADC1 data
  for (size_t i = 0; i < adc1_data_.size(); i++) {
    const auto &data = adc1_data_[i];
    if (!first)
      ss << ",";

    auto idx = (data.write_index > 0) ? (data.write_index - 1) : (config_.adc_history_size - 1);
    int voltage_mv = data.values[idx];

    // Key format: "1_5" for ADC1 channel 5
    ss << "\"1_" << static_cast<int>(data.channel) << "\":{";
    ss << "\"voltage\":" << voltage_mv << ",";
    ss << "\"current\":" << voltage_mv << ",";
    ss << "\"label\":\""
       << (data.label.empty() ? ("ADC1_CH" + std::to_string(static_cast<int>(data.channel)))
                              : data.label)
       << "\"";
    ss << "}";
    first = false;
  }

  // ADC2 data
  for (size_t i = 0; i < adc2_data_.size(); i++) {
    const auto &data = adc2_data_[i];
    if (!first)
      ss << ",";

    auto idx = (data.write_index > 0) ? (data.write_index - 1) : (config_.adc_history_size - 1);
    int voltage_mv = data.values[idx];

    // Key format: "2_5" for ADC2 channel 5
    ss << "\"2_" << static_cast<int>(data.channel) << "\":{";
    ss << "\"voltage\":" << voltage_mv << ",";
    ss << "\"current\":" << voltage_mv << ",";
    ss << "\"label\":\""
       << (data.label.empty() ? ("ADC2_CH" + std::to_string(static_cast<int>(data.channel)))
                              : data.label)
       << "\"";
    ss << "}";
    first = false;
  }

  ss << "}";
  return ss.str();
}
