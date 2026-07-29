#include "gps.hpp"

#include <cstring>

using namespace espp;

Gps::Gps(const Config &config)
    : BaseComponent("Gps", config.log_level)
    , config_(config) {
  if (config_.auto_start) {
    std::error_code ec;
    if (!start(ec)) {
      logger_.error("Failed to start: {}", ec.message());
    }
  }
}

Gps::~Gps() { stop(); }

bool Gps::start(std::error_code &ec) {
  if (running_) {
    logger_.warn("Already running");
    return true;
  }
  if (config_.rx_io_num == GPIO_NUM_NC) {
    logger_.error("No RX pin configured");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  logger_.info("Starting GPS on UART{} (rx={}, tx={}, {} baud)", (int)config_.uart_port,
               (int)config_.rx_io_num, (int)config_.tx_io_num, config_.baud_rate);
  uart_config_t uart_config;
  memset(&uart_config, 0, sizeof(uart_config));
  uart_config.baud_rate = (int)config_.baud_rate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;
  esp_err_t err =
      uart_driver_install(config_.uart_port, (int)config_.rx_buffer_size, 256, 0, nullptr, 0);
  if (err != ESP_OK) {
    logger_.error("Failed to install UART driver: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  uart_installed_ = true;
  err = uart_param_config(config_.uart_port, &uart_config);
  if (err == ESP_OK) {
    err = uart_set_pin(config_.uart_port, config_.tx_io_num, config_.rx_io_num, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
  }
  if (err != ESP_OK) {
    logger_.error("Failed to configure UART: {}", esp_err_to_name(err));
    uart_driver_delete(config_.uart_port);
    uart_installed_ = false;
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  line_buffer_.clear();
  line_buffer_.reserve(128);
  running_ = true;
  task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = espp::Task::callback_m_cv_fn(
          [this](std::mutex &m, std::condition_variable &cv) { return read_task(m, cv); }),
      .task_config = config_.task_config,
      .log_level = espp::Logger::Verbosity::WARN});
  task_->start();
  return true;
}

bool Gps::stop() {
  if (!running_) {
    return false;
  }
  running_ = false;
  task_.reset();
  if (uart_installed_) {
    uart_driver_delete(config_.uart_port);
    uart_installed_ = false;
  }
  return true;
}

GpsFix Gps::fix() const {
  std::lock_guard<std::mutex> lock(fix_mutex_);
  return parser_.fix();
}

bool Gps::has_fix() const {
  std::lock_guard<std::mutex> lock(fix_mutex_);
  return parser_.fix().valid;
}

bool Gps::write(std::string_view data, std::error_code &ec) {
  if (!uart_installed_ || config_.tx_io_num == GPIO_NUM_NC) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }
  int written = uart_write_bytes(config_.uart_port, data.data(), data.size());
  if (written != (int)data.size()) {
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  return true;
}

bool Gps::read_task(std::mutex &m, std::condition_variable &cv) {
  uint8_t buffer[256];
  int len = uart_read_bytes(config_.uart_port, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
  if (len <= 0) {
    return false; // nothing received; don't stop the task
  }
  for (int i = 0; i < len; i++) {
    char c = (char)buffer[i];
    if (c == '\n') {
      if (!line_buffer_.empty()) {
        handle_line(line_buffer_);
        line_buffer_.clear();
      }
    } else if (c != '\r') {
      // protect against garbage / binary data filling memory
      if (line_buffer_.size() < 120) {
        line_buffer_.push_back(c);
      } else {
        line_buffer_.clear();
      }
    }
  }
  return false; // don't stop the task
}

void Gps::handle_line(std::string_view line) {
  logger_.debug("NMEA: {}", line);
  if (!NmeaParser::checksum_valid(line)) {
    return;
  }
  if (config_.on_sentence) {
    config_.on_sentence(line);
  }
  bool is_rmc = line.size() > 6 && line.substr(3, 3) == "RMC";
  GpsFix fix_copy;
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    // checksum already validated above; parse_unchecked avoids recomputing it
    parser_.parse_unchecked(line);
    fix_copy = parser_.fix();
  }
  // RMC is the last sentence of interest in a typical update cycle, so use
  // it as the "fix updated" event
  if (is_rmc && config_.on_fix) {
    config_.on_fix(fix_copy);
  }
}
