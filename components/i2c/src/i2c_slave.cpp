#include <sdkconfig.h>

// Only compile this file if the new API is selected
#if defined(CONFIG_ESPP_I2C_USE_NEW_API)

#include "i2c_slave.hpp"

namespace espp {

I2cSlaveDevice::I2cSlaveDevice(const Config &config)
    : BaseComponent("I2cSlaveDevice", config.log_level)
    , config_(config) {}

I2cSlaveDevice::~I2cSlaveDevice() {
  std::error_code ec;
  deinit(ec);
}

bool I2cSlaveDevice::init(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (initialized_) {
    ec = std::make_error_code(std::errc::already_connected);
    return false;
  }
  i2c_slave_config_t slave_cfg = {};
  slave_cfg.i2c_port = config_.port;
  slave_cfg.sda_io_num = static_cast<gpio_num_t>(config_.sda_io_num);
  slave_cfg.scl_io_num = static_cast<gpio_num_t>(config_.scl_io_num);
  slave_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  slave_cfg.slave_addr = config_.slave_address;
  slave_cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
  slave_cfg.send_buf_depth = 128;
#if CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2
  slave_cfg.flags.enable_internal_pullup = config_.enable_internal_pullup;
#endif
  slave_cfg.flags.allow_pd = 0;
  esp_err_t err = i2c_new_slave_device(&slave_cfg, &dev_handle_);
  if (err != ESP_OK) {
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  initialized_ = true;
  ec.clear();
  return true;
}

bool I2cSlaveDevice::deinit(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_)
    return true;
  if (dev_handle_) {
    esp_err_t err = i2c_del_slave_device(dev_handle_);
    if (err != ESP_OK) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    dev_handle_ = nullptr;
  }
  initialized_ = false;
  ec.clear();
  return true;
}

bool I2cSlaveDevice::write(const uint8_t *data, size_t len, std::error_code &ec) {
#if CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !dev_handle_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  uint32_t write_len = 0;
  esp_err_t err = i2c_slave_write(dev_handle_, data, len, &write_len, -1);
  if (err != ESP_OK || write_len != len) {
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
#else
  logger_.error("I2cSlaveDevice::write() is not supported in this configuration. Please ENABLE "
                "CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2.");
  ec = std::make_error_code(std::errc::not_supported);
  return false;
#endif
}

bool I2cSlaveDevice::read(uint8_t *data, size_t len, std::error_code &ec) {
#if !CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !dev_handle_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  esp_err_t err = i2c_slave_receive(dev_handle_, data, len);
  if (err != ESP_OK) {
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
#else
  logger_.error("I2cSlaveDevice::read() is not supported in this configuration. Please DISABLE "
                "CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2.");
  ec = std::make_error_code(std::errc::not_supported);
  return false;
#endif
}

bool I2cSlaveDevice::register_callbacks(const Callbacks &cb, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  callbacks_ = cb;
  // TODO: Implement registration with ESP-IDF and dispatch in task context
  ec.clear();
  return true;
}

} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_NEW_API
