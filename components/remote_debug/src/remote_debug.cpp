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

  // Initialize GPIO
  for (const auto &gpio : config_.gpios) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << gpio.pin);
    io_conf.mode = gpio.mode;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_map_[gpio.pin] = gpio;
  }

  // Initialize ADC
  bool need_adc1 = false, need_adc2 = false;
  for (const auto &adc : config_.adcs) {
    if (adc.unit == ADC_UNIT_1)
      need_adc1 = true;
    if (adc.unit == ADC_UNIT_2)
      need_adc2 = true;
    adc_map_[{adc.unit, adc.channel}] = adc;
  }

  if (need_adc1) {
    adc_oneshot_unit_init_cfg_t adc1_config = {.unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&adc1_config, &adc1_handle_);
  }

  if (need_adc2) {
    adc_oneshot_unit_init_cfg_t adc2_config = {.unit_id = ADC_UNIT_2};
    adc_oneshot_new_unit(&adc2_config, &adc2_handle_);
  }

  // Configure ADC channels
  for (const auto &[key, adc] : adc_map_) {
    adc_oneshot_chan_cfg_t chan_config = {.atten = adc.atten, .bitwidth = ADC_BITWIDTH_DEFAULT};
    auto handle = (adc.unit == ADC_UNIT_1) ? adc1_handle_ : adc2_handle_;
    if (handle) {
      adc_oneshot_config_channel(handle, adc.channel, &chan_config);
    }

    // Initialize data storage
    adc_data_[key].values.resize(config_.adc_history_size, 0);
    adc_data_[key].timestamps.resize(config_.adc_history_size, 0);
  }

  logger_.info("RemoteDebug initialized with {} GPIOs and {} ADCs", config_.gpios.size(),
               config_.adcs.size());
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
  if (!adc_map_.empty()) {
    sampling_active_ = true;
    sampling_thread_ = std::make_unique<std::thread>([this]() { adc_sampling_task(); });
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

  stop_server();
  is_active_ = false;
  logger_.info("RemoteDebug stopped");
}

bool RemoteDebug::set_gpio(gpio_num_t pin, int level) {
  if (gpio_map_.find(pin) == gpio_map_.end()) {
    logger_.error("GPIO {} not configured", static_cast<int>(pin));
    return false;
  }
  gpio_set_level(pin, level);
  return true;
}

int RemoteDebug::get_gpio(gpio_num_t pin) {
  if (gpio_map_.find(pin) == gpio_map_.end()) {
    logger_.error("GPIO {} not configured", static_cast<int>(pin));
    return -1;
  }
  return gpio_get_level(pin);
}

int RemoteDebug::read_adc(adc_unit_t unit, adc_channel_t channel) {
  auto key = std::make_pair(unit, channel);
  if (adc_map_.find(key) == adc_map_.end()) {
    return -1;
  }

  int raw_value = 0;
  auto handle = (unit == ADC_UNIT_1) ? adc1_handle_ : adc2_handle_;
  if (handle && adc_oneshot_read(handle, channel, &raw_value) == ESP_OK) {
    return raw_value;
  }
  return -1;
}

void RemoteDebug::adc_sampling_task() {
  while (sampling_active_) {
    auto now = esp_timer_get_time();

    std::lock_guard<std::mutex> lock(adc_mutex_);
    for (auto &[key, data] : adc_data_) {
      int value = read_adc(key.first, key.second);
      if (value >= 0) {
        data.values[data.write_index] = value;
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

  httpd_uri_t adc_get_uri = {
      .uri = "/api/adc/get", .method = HTTP_GET, .handler = adc_get_handler, .user_ctx = this};
  httpd_register_uri_handler(server_, &adc_get_uri);

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

esp_err_t RemoteDebug::adc_get_handler(httpd_req_t *req) {
  auto *self = static_cast<RemoteDebug *>(req->user_ctx);

  // Parse query string for unit and channel
  char query[100];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    int unit = -1, channel = -1;
    char unit_str[16], channel_str[16];
    if (httpd_query_key_value(query, "unit", unit_str, sizeof(unit_str)) == ESP_OK) {
      unit = atoi(unit_str);
    }
    if (httpd_query_key_value(query, "channel", channel_str, sizeof(channel_str)) == ESP_OK) {
      channel = atoi(channel_str);
    }

    if (unit >= 0 && channel >= 0) {
      int value =
          self->read_adc(static_cast<adc_unit_t>(unit), static_cast<adc_channel_t>(channel));
      char resp[32];
      snprintf(resp, sizeof(resp), "{\"value\":%d}", value);
      httpd_resp_set_type(req, "application/json");
      httpd_resp_send(req, resp, strlen(resp));
      return ESP_OK;
    }
  }

  httpd_resp_send_500(req);
  return ESP_FAIL;
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
  ss << R"(<!DOCTYPE html><html><head><title>Remote Debug</title>
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
<h1>&#x1F527; Remote Debug Interface</h1>)";

  // GPIO controls
  ss << "<div class='card'><h2>GPIO Control</h2>";
  ss << "<div style='display: grid; grid-template-columns: 2fr 1fr 1fr; gap: 10px; padding: 10px; "
        "font-weight: bold; border-bottom: 2px solid #667eea; margin-bottom: 15px;'>";
  ss << "<div>Pin</div><div style='text-align: center;'>Controls</div><div style='text-align: "
        "center;'>State</div>";
  ss << "</div>";
  for (const auto &[pin, cfg] : gpio_map_) {
    ss << "<div style='display: grid; grid-template-columns: 2fr 1fr 1fr; gap: 10px; padding: "
          "15px; margin: 10px 0; background: #f8f9fa; border-radius: 10px; align-items: center;'>";
    ss << "<span class='gpio-label'>" << cfg.label << " (GPIO " << pin << ")</span>";
    ss << "<div style='display: flex; gap: 10px; justify-content: center;'>";
    ss << "<button class='btn-high' onclick='setGpio(" << pin << ",1)'>HIGH</button>";
    ss << "<button class='btn-low' onclick='setGpio(" << pin << ",0)'>LOW</button>";
    ss << "</div>";
    ss << "<span class='gpio-state' id='gpio" << pin << "' style='text-align: center;'>?</span>";
    ss << "</div>";
  }
  ss << "</div>";

  // ADC display
  if (!adc_map_.empty()) {
    ss << "<div class='card'><h2>ADC Monitoring</h2>";
    for (const auto &[key, cfg] : adc_map_) {
      ss << "<div class='adc'>";
      ss << "<span class='adc-label'>" << cfg.label << " (Unit:" << key.first
         << " Ch:" << key.second << ")</span>";
      ss << "<span class='adc-value' id='adc" << key.first << "_" << key.second << "'>?</span>";
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

function updateAdc() {
  fetch('/api/adc/data')
    .then(r => r.json())
    .then(data => {
      const now = Date.now();
      for (let key in data) {
        const elem = document.getElementById('adc' + key);
        if (elem) {
          const volts = (data[key].voltage / 1000.0).toFixed(3);
          elem.innerText = volts + ' V (' + data[key].current + ' raw)';
        }
        
        if (!adcHistory[key]) {
          adcHistory[key] = { values: [], times: [], color: colors[colorIndex++ % colors.length] };
        }
        
        adcHistory[key].values.push(data[key].current);
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
  
  // Find min/max across all series
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
    
    const val = Math.round(maxVal - (range * i / 4));
    ctx.fillStyle = '#666';
    ctx.font = '12px Arial';
    ctx.fillText(val, 5, y + 4);
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
    ctx.fillStyle = adcHistory[key].color;
    ctx.fillRect(padding + chartWidth - 100, legendY, 15, 15);
    ctx.fillStyle = '#333';
    ctx.font = '12px Arial';
    ctx.fillText('ADC ' + key, padding + chartWidth - 80, legendY + 12);
    legendY += 20;
  }
}

setInterval(updateGpio, 1000);
setInterval(updateAdc, 200);
updateGpio();
updateAdc();
</script></div></body></html>)";

  return ss.str();
}

std::string RemoteDebug::get_gpio_state_json() const {
  std::stringstream ss;
  ss << "{";
  bool first = true;
  for (const auto &[pin, cfg] : gpio_map_) {
    if (!first)
      ss << ",";
    ss << "\"" << pin << "\":" << gpio_get_level(pin);
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
  for (const auto &[key, data] : adc_data_) {
    if (!first)
      ss << ",";
    auto idx = (data.write_index > 0) ? (data.write_index - 1) : (config_.adc_history_size - 1);
    int raw_value = data.values[idx];
    // Convert raw ADC to millivolts
    float voltage_mv = 0.0f;
    // if (adcs_.count(key) > 0) {
    //   voltage_mv = adcs_.at(key)->read_mv();
    // }
    ss << "\"" << key.first << "_" << key.second << "\":{";
    ss << "\"current\":" << raw_value << ",";
    ss << "\"voltage\":" << std::fixed << std::setprecision(3) << voltage_mv << "}";
    first = false;
  }
  ss << "}";
  return ss.str();
}
